#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import trimesh
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from scipy.spatial import ConvexHull
from tf.transformations import euler_from_quaternion
import supervision as sv


def inv_T(T):
    """Invert a rigid 4x4 transform."""
    Rm = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = Rm.T
    Ti[:3, 3] = -Rm.T @ t
    return Ti

def pose_to_T(pose):
    """
    geometry_msgs/Pose -> 4x4 homogeneous transform.

    IMPORTANT FRAME NOTE:
      Given a pose that is "the pose of frame B expressed in frame A" (e.g. Gazebo ModelStates pose
      is model pose w.r.t. world), this function returns the transform that maps coordinates
      FROM the pose's local frame (B) TO the reference frame (A).

      In symbols (common robotics notation):
        pose gives:   position t_A_B and orientation R_A_B
        returned T is:   ^A T_B  (maps p_B -> p_A)

      Concretely, applying the returned T does:
        p_A = R_A_B * p_B + t_A_B

    ROS quaternion order is (x, y, z, w).
    """
    t = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

    q = np.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ], dtype=float)

    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Quaternion norm is ~0")
    q = q / n

    Rm = R.from_quat(q).as_matrix()  # expects [x, y, z, w]
    
    T = np.eye(4, dtype=float)
    T[:3, :3] = Rm
    T[:3, 3] = t
    return T


def apply_T(V, T):
    """
    Apply a 4x4 transform to Nx3 vertices.

    FRAME NOTE:
      If V are points expressed in frame B (p_B), and T is ^A T_B,
      then apply_T(V, T) returns points expressed in frame A (p_A).
    """
    Vh = np.hstack([V, np.ones((V.shape[0], 1), dtype=float)])
    Vw = (T @ Vh.T).T
    return Vw[:, :3]


def project_to_plane_uv(points_w, plane_origin_w, plane_R_w):
    """
    Project 3D points in WORLD frame onto a plane frame and return 2D coordinates.

    points_w: Nx3 points expressed in world frame W (p_W)
    plane_origin_w: 3-vector, a point on plane expressed in W
    plane_R_w: 3x3 rotation whose COLUMNS are plane basis axes expressed in W:
        u_W = plane_R_w[:,0]  (plane +x axis expressed in W)
        v_W = plane_R_w[:,1]  (plane +y axis expressed in W)
        n_W = plane_R_w[:,2]  (plane normal expressed in W)

    Returns:
      uv: Nx2 coordinates in the plane basis (units match world, typically meters)
      proj_w: Nx3 orthogonal projection points expressed in world W
    """
    u = plane_R_w[:, 0]  # plane x axis in world
    v = plane_R_w[:, 1]  # plane y axis in world
    n = plane_R_w[:, 2]  # plane normal in world

    # ensure unit basis (defensive)
    u = u / np.linalg.norm(u)
    v = v / np.linalg.norm(v)
    n = n / np.linalg.norm(n)

    r = points_w - plane_origin_w[None, :]

    # 2D coordinates in plane basis:
    # x = dot(r_W, u_W), y = dot(r_W, v_W)
    x = r @ u
    y = r @ v
    uv = np.stack([x, y], axis=1)

    # orthogonal projection back onto plane (optional)
    dist = r @ n
    proj_w = points_w - dist[:, None] * n[None, :]

    return uv, proj_w


def bbox_from_uv(uv):
    """Axis-aligned bbox in plane coords (min/max) in the plane's 2D (u,v) coordinates."""
    mn = uv.min(axis=0)
    mx = uv.max(axis=0)
    corners_uv = np.array([
        [mn[0], mn[1]],
        [mx[0], mn[1]],
        [mx[0], mx[1]],
        [mn[0], mx[1]],
    ], dtype=float)
    return mn, mx, corners_uv


def uv_to_world(corners_uv, plane_origin_w, plane_R_w):
    """
    Convert Nx2 uv points on plane back to world coordinates.

    FRAME NOTE:
      corners_uv are coordinates in the plane's local 2D basis (u,v).
      The returned points are expressed in world frame W.
    """
    u = plane_R_w[:, 0]
    v = plane_R_w[:, 1]
    return plane_origin_w[None, :] + corners_uv[:, 0:1] * u[None, :] + corners_uv[:, 1:2] * v[None, :]


def uv_to_image_mask_convex(uv, width=640, height=480, margin=0):
    """
    uv: Nx2 points in PLANE coordinates (u,v)

    This function constructs a *separate* 2D pixel coordinate system:
      - image x increases right
      - image y increases downward (standard image coordinates)
    and builds an affine mapping from (u,v) -> (px,py) by fitting the uv-bbox into the image.

    Returns:
      mask (H,W) uint8 with 0 background, 255 filled object
      T: dict with mapping info (so you can map pixels back to uv if desired)
    """
    uv = np.asarray(uv, dtype=float)
    if uv.shape[0] < 3:
        mask = np.zeros((height, width), dtype=np.uint8)
        return mask, None

    # Convex hull in uv
    hull = ConvexHull(uv)
    poly_uv = uv[hull.vertices]  # Mx2 in CCW order (in uv space)

    # Compute bbox in uv and build an affine mapping uv->pixel
    mn = poly_uv.min(axis=0)
    mx = poly_uv.max(axis=0)
    span = np.maximum(mx - mn, 1e-9)

    # Keep aspect ratio: fit the uv bbox into image with margin
    sx = (width  - 2 * margin) / span[0]
    sy = (height - 2 * margin) / span[1]
    s = min(sx, sy)

    # Map: pixel = [ (u - mn_u)*s + margin, (v - mn_v)*s + margin ]
    # NOTE: this does NOT flip v. If you want plane +v to map upward in the image,
    # you can flip v into image coordinates (y down) using the commented alternative.
    def uv_to_px(pts_uv):
        pts = (pts_uv - mn[None, :]) * s
        pts[:, 0] += margin
        pts[:, 1] += margin
        return pts

    # def uv_to_px(pts_uv):
    #     pts = (pts_uv - mn[None, :]) * s
    #     pts[:, 0] += margin
    #     pts[:, 1] = (height - 1) - (pts[:, 1] + margin)  # flip v for image coordinates
    #     return pts

    poly_px = uv_to_px(poly_uv)

    # Convert to int pixels
    poly_px_i = np.round(poly_px).astype(np.int32)

    # Create mask and fill polygon
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [poly_px_i], 255)

    T = {
        "mn_uv": mn,
        "scale": s,
        "margin": margin,
        "width": width,
        "height": height,
        "flip_v": False,  # set True if you implement the v flip
    }
    return mask, T


def draw_bbox_on_mask(mask, bbox_corners_uv, T, thickness=2):
    """
    Optional: draw axis-aligned bbox (in plane coords) on the image.
    bbox_corners_uv: 4x2 in uv coords (plane coords)
    T: mapping dict returned by uv_to_image_mask_convex (uv->pixel mapping)
    """
    if T is None:
        return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    mn = T["mn_uv"]
    s = T["scale"]
    margin = T["margin"]
    H, W = mask.shape[:2]

    bbox_uv = np.asarray(bbox_corners_uv, dtype=float)

    pts = (bbox_uv - mn[None, :]) * s

    pts[:, 0] += margin
    pts[:, 1] += margin
    pts_i = np.round(pts).astype(np.int32)

    img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.polylines(img, [pts_i], isClosed=True, color=(255, 255, 255), thickness=thickness)
    return img




def debug_visualize_scene(mesh, T_obj_w, T_cam_w, plane_origin_w=None, plane_R_w=None):
    """
    Visualize:
      - mesh transformed by object pose
      - world frame W
      - object frame O
      - camera frame C
      - projection plane frame P (optional)

    FRAME NOTE:
      This function assumes:
        T_obj_w is ^W T_O  (object -> world)
        T_cam_w is ^W T_C  (camera -> world)

      trimesh.creation.axis(transform=...) expects a transform that places the axis geometry
      into the scene/world, i.e. a "frame -> world" placement.
    """

    scene = trimesh.Scene()

    # -------------------------------------------------
    # 2) World frame W
    # -------------------------------------------------
    world_axis = trimesh.creation.axis(origin_size=0.05)
    scene.add_geometry(world_axis)

    # -------------------------------------------------
    # 4) Camera frame C placed in world using ^W T_C
    # -------------------------------------------------
    cam_axis = trimesh.creation.axis(
        transform=T_cam_w,
        origin_size=0.1
    )
    scene.add_geometry(cam_axis)

    # -------------------------------------------------
    # 1) Transformed mesh (object mesh in world)
    #    Apply ^W T_O to mesh vertices in object/mesh-local coords
    # -------------------------------------------------
    mesh_vis = mesh.copy()
    V = mesh.vertices
    Vh = np.hstack([V, np.ones((V.shape[0], 1))])
    Vw = (T_obj_w @ Vh.T).T[:, :3]
    mesh_vis.vertices = Vw
    mesh_vis.visual.face_colors = [180, 180, 255, 150]  # light blue
    scene.add_geometry(mesh_vis)

    # -------------------------------------------------
    # 3) Object frame O placed in world using ^W T_O
    # -------------------------------------------------
    obj_axis = trimesh.creation.axis(
        transform=T_obj_w,
        origin_size=0.08
    )
    scene.add_geometry(obj_axis)

    # -------------------------------------------------
    # 5) Projection plane (if provided), expressed/placed in world W
    # -------------------------------------------------
    if plane_origin_w is not None and plane_R_w is not None:
        plane_size = 4.0  # adjust if needed

        # Create square in local plane coordinates (u,v,0) then map into world
        square = np.array([
            [-plane_size, -plane_size, 0],
            [ plane_size, -plane_size, 0],
            [ plane_size,  plane_size, 0],
            [-plane_size,  plane_size, 0],
        ])

        # Transform square to world:
        # p_W = origin_W + u*u_W + v*v_W
        square_w = (
            plane_origin_w[None, :]
            + square[:, 0:1] * plane_R_w[:, 0][None, :]
            + square[:, 1:2] * plane_R_w[:, 1][None, :]
        )

        faces = np.array([[0,1,2], [0,2,3]])
        plane_mesh = trimesh.Trimesh(vertices=square_w, faces=faces)
        plane_mesh.visual.face_colors = [255, 0, 0, 100]
        scene.add_geometry(plane_mesh)

        # -------------------------------------------------
        # 6) Plane normal (plane +z axis expressed in world)
        # -------------------------------------------------
        normal = plane_R_w[:, 2]
        line = np.vstack([
            plane_origin_w,
            plane_origin_w + normal * plane_size 
        ])

        normal_path = trimesh.load_path(line)
        scene.add_geometry(normal_path)

    scene.show()


# ----------------------------
# NEW: pinhole projection tools
# ----------------------------

def bbox_from_pixels(uv_pix):
    """Axis-aligned bbox from Nx2 pixels: returns (umin,vmin,umax,vmax)."""
    umin = float(np.min(uv_pix[:, 0]))
    vmin = float(np.min(uv_pix[:, 1]))
    umax = float(np.max(uv_pix[:, 0]))
    vmax = float(np.max(uv_pix[:, 1]))
    return umin, vmin, umax, vmax

def project_mesh_pinhole(V_obj, R_co, t_co, fx, fy, cx, cy, clip_z=1e-6):
    """
    Project mesh vertices (object frame) into image pixels using pinhole camera.

    V_obj: Nx3 in object frame
    R_co: 3x3 rotation object->camera
    t_co: 3-vector translation of object origin expressed in camera frame
    Returns:
      pix: Mx2 pixel projections (only vertices with Z>clip_z)
      Z:   M depths
    """
    Vc = (R_co @ V_obj.T).T + t_co[None, :]
    Z = Vc[:, 2]
    mask = Z > clip_z
    if not np.any(mask):
        return np.zeros((0, 2), dtype=float), np.zeros((0,), dtype=float)

    Vc = Vc[mask]
    Z = Vc[:, 2]
    u = fx * (Vc[:, 0] / Z) + cx
    v = fy * (Vc[:, 1] / Z) + cy
    pix = np.stack([u, v], axis=1)
    return pix, Z

def ray_from_pixel(u0, v0, fx, fy, cx, cy, normalize=True):
    """
    Back-project pixel to a ray in camera coordinates.
    """
    r = np.array([(u0 - cx) / fx, (v0 - cy) / fy, 1.0], dtype=float)
    if normalize:
        r /= np.linalg.norm(r)
    return r

def solve_depth_from_bbox_scaling(V_obj, R_co, fx, fy, cx, cy, w_obs, h_obs,
                                 d_ref=8.0, use_max=True):
    """
    Fast depth estimate using reference projection scaling ~ 1/depth.

    Returns d0, and also (w_ref, h_ref) for debugging.
    """
    # reference: object origin on optical axis at d_ref
    t_ref = np.array([0.0, 0.0, d_ref], dtype=float)
    pix_ref, _ = project_mesh_pinhole(V_obj, R_co, t_ref, fx, fy, cx, cy)
    if pix_ref.shape[0] < 3:
        return None, None, None

    umin, vmin, umax, vmax = bbox_from_pixels(pix_ref)
    w_ref = umax - umin
    h_ref = vmax - vmin

    if w_obs <= 1e-6 or h_obs <= 1e-6:
        return None, w_ref, h_ref

    d_w = d_ref * (w_ref / w_obs)
    d_h = d_ref * (h_ref / h_obs)

    d0 = max(d_w, d_h) if use_max else 0.5 * (d_w + d_h)
    return float(d0), float(w_ref), float(h_ref)

def refine_depth_1d(V_obj, R_co, r_cam, fx, fy, cx, cy, w_obs, h_obs,
                    d_init, n_steps=40, step_scale=2.0):
    """
    Simple 1D refinement by searching around d_init on a log-spaced bracket.
    This avoids needing scipy.optimize and stays robust.

    We minimize relative squared error of (w,h).
    """
    if d_init is None or d_init <= 1e-9:
        return None

    # bracket around d_init: [d_init/step_scale, d_init*step_scale]
    d_lo = d_init / step_scale
    d_hi = d_init * step_scale

    # log-spaced candidates
    ds = np.exp(np.linspace(np.log(d_lo), np.log(d_hi), n_steps))

    best_d = None
    best_E = np.inf

    for d in ds:
        t = d * r_cam
        pix, _ = project_mesh_pinhole(V_obj, R_co, t, fx, fy, cx, cy)
        if pix.shape[0] < 3:
            continue
        umin, vmin, umax, vmax = bbox_from_pixels(pix)
        w = umax - umin
        h = vmax - vmin

        # relative error (robust to scale)
        ew = (w - w_obs) / max(w_obs, 1e-6)
        eh = (h - h_obs) / max(h_obs, 1e-6)
        E = ew*ew + eh*eh

        if E < best_E:
            best_E = E
            best_d = d

    return float(best_d) if best_d is not None else None




class DaeProjector:
    def __init__(self):
        

        self.mesh_path = "/home/pgoldschmid/src/test_apriltags/src/anafi_sim/sphinx_gazebo_ws/src/anafi_ros/src/olympe_bridge/conv_hull_blimp.dae"



        # Load DAE mesh
        m = trimesh.load_mesh(self.mesh_path, filetype="dae")
        if not isinstance(m, trimesh.Trimesh):
            # if a Scene is returned, concatenate
            if hasattr(m, "dump"):
                parts = m.dump()
                if len(parts) == 0:
                    raise RuntimeError("DAE contained no mesh geometry")
                m = trimesh.util.concatenate(parts)
            else:
                raise RuntimeError("Unsupported trimesh load output for this DAE")
        # m.apply_transform(np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]))


        #Scale for 3D printed blimp for testing
        # m.apply_scale(0.022)




        bounds = m.bounds              # shape (2, 3): [min; max]
        center = bounds.mean(axis=0)           # (min + max) / 2

        # Translate mesh so that AABB center moves to origin
        m.apply_translation(-center)
        self.mesh = m
        self.center = center

        self.V0 = np.asarray(self.mesh.vertices, dtype=float)

        self._obj_pose_stamped = None
        self._cam_pose_stamped = None
        self.cvbridge = CvBridge()        
        self.img = np.zeros((3,3,3))
        self.img_msg = self.cvbridge.cv2_to_imgmsg(self.img, encoding="passthrough")

        self.p_wo_from_cam = np.zeros(3)

        self.width = 0
        self.height = 0


        self.latest = {
            "uv": None,
            "bbox_min": None,
            "bbox_max": None,
            "bbox_corners_uv": None,
            "bbox_corners_world": None,
            "plane_origin_world": None,
            "plane_R_world": None,
        }


        # ---------------------------------------------
        # NEW: camera intrinsics (set these correctly!)
        # ---------------------------------------------
        # Init values, need to be overwritten with real camera values
        
        self.fx = 2793.6089081858354
        self.fy = 2793.6089081858354
        self.cx = 1920.5
        self.cy = 1080.5

        # ---------------------------------------------------
        # NEW: YOLO bbox input (set/update from your detector)
        # ---------------------------------------------------
        # Format: (umin, vmin, umax, vmax) in pixels
        self.yolo_bbox = None

        rospy.loginfo("Dae mesh object module: Loaded mesh vertices: %d", self.V0.shape[0])
        return

    def set_camera_parameters(self,fx,fy,cx,cy,width,height):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height
        return

    def set_yolo_bbox(self, umin, vmin, umax, vmax):
        self.yolo_bbox = (float(umin), float(vmin), float(umax), float(vmax))
        self.try_compute()

    @property
    def obj_pose_stamped(self):
        return self._obj_pose_stamped
    
    @property
    def cam_pose_stamped(self):
        return self._cam_pose_stamped

    @obj_pose_stamped.setter
    def obj_pose_stamped(self,value):
        old = self._obj_pose_stamped
        self._obj_pose_stamped = value
        r,p,y = np.rad2deg(euler_from_quaternion(np.array([self._obj_pose_stamped.pose.orientation.x,self._obj_pose_stamped.pose.orientation.y,self._obj_pose_stamped.pose.orientation.z,self._obj_pose_stamped.pose.orientation.w])))
        # print("dae",r,p,y)
        # if value != old:
        #     self.try_compute()
        return 

    @cam_pose_stamped.setter
    def cam_pose_stamped(self,value):
        old = self._cam_pose_stamped
        self._cam_pose_stamped = value
        # if value != old:
        #     self.try_compute()
        return 


    def try_compute(self):
        if self._obj_pose_stamped is None or self._cam_pose_stamped is None:
            rospy.loginfo_throttle(1.0, "Compute skipped: missing obj or cam pose")
            return

        img = self.out_cv

        if self.yolo_bbox is None:
            rospy.loginfo(1.0, "Compute skipped: missing YOLO bbox")
            cv2.putText(img,"Missing bbox!", 
                (int(0),int(0)+200), 
                cv2.FONT_HERSHEY_SIMPLEX, #font
                4,  #font scale
                (255,0,0), # font color
                5) #line thickness
            self.img_msg = self.cvbridge.cv2_to_imgmsg(img, encoding="rgb8")
            return
        
        

        

        # ^W T_O and ^W T_C
        T_obj_w = pose_to_T(self._obj_pose_stamped.pose)
        T_cam_w = pose_to_T(self._cam_pose_stamped.pose)

        p_cam_w = T_cam_w[:3,3]

        # Your camera-frame correction (keep if it matches your actual optical frame)
        R_y90 = np.array([
            [0.0, 0.0,  1.0, 0.0],
            [0.0, 1.0,  0.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0,  0.0, 1.0]
        ], dtype=float)

        Rz_minus_90 = np.array([
            [0.0,  1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0,  0.0, 1.0, 0.0],
            [0.0,  0.0, 0.0, 1.0]
        ], dtype=float)

        T_cam_w = T_cam_w @ R_y90 @ Rz_minus_90

        # -------------------------------
        # NEW: compute relative rotation
        # -------------------------------
        R_wo = T_obj_w[:3, :3]  # object->world
        R_wc = T_cam_w[:3, :3]  # camera->world
        R_cw = R_wc.T           # world->camera
        R_co = R_cw @ R_wo      # object->camera


        # -------------------------------
        # YOLO bbox -> center + size
        # -------------------------------
        umin, vmin, umax, vmax = self.yolo_bbox
        u0 = 0.5 * (umin + umax)
        v0 = 0.5 * (vmin + vmax)
        w_obs = umax - umin
        h_obs = vmax - vmin

        



        #Check if yolo box is actually reasonable. If not, skip computation.
        #Get reference aspect ration of bounding box aspect_ratio = w_obs / h_obs
        # aspect_ratio_yolo = w_obs / h_obs
# 
        # pix, _ = project_mesh_pinhole(self.V0, R_co, np.array([0,0,10]), self.fx, self.fy, self.cx, self.cy)
        # umin_p_ref, vmin_p_ref, umax_p_ref, vmax_p_ref = bbox_from_pixels(pix)
        # w_ref = umax_p_ref - umin_p_ref 
        # h_ref = vmax_p_ref - vmin_p_ref 
        # aspect_ratio_ref = w_ref / h_ref
        # print("asp_rat_yo =",aspect_ratio_yolo)
        # print("asp_rat_re =",aspect_ratio_ref)
        # print( "deviation =",np.abs(aspect_ratio_yolo - aspect_ratio_ref))
        # print("---")

        margin = 10
        # print("umin =",umin)
        # print("umax =",umax)
        # print("vmin =",vmin)
        # print("vmax =",vmax)
        # print("widt =",self.width)
        # print("heig =",self.height)
        # print("---")
        if umin < margin or umax > self.width-margin or vmin < margin or vmax > self.height-margin:
            rospy.loginfo_throttle(1.0, "Compute skipped: YOLO bbox invalid")
            cv2.putText(img,"Invalid bbox!", 
                (int(umin),int(vmax)+200), 
                cv2.FONT_HERSHEY_SIMPLEX, #font
                4,  #font scale
                (255,0,0), # font color
                5) #line thickness
            self.img_msg = self.cvbridge.cv2_to_imgmsg(img, encoding="rgb8")

            return



        



        # Ray through bbox center in camera frame
        r_cam = ray_from_pixel(u0, v0, self.fx, self.fy, self.cx, self.cy, normalize=True)

        # ---------------------------------------------------------
        # Solve depth along ray so projected mesh bbox matches YOLO
        # ---------------------------------------------------------
        d0, w_ref, h_ref = solve_depth_from_bbox_scaling(
            self.V0, R_co, self.fx, self.fy, self.cx, self.cy,
            w_obs=w_obs, h_obs=h_obs,
            d_ref=8.0, use_max=True
        )

        if d0 is None:
            rospy.logwarn_throttle(1.0, "Depth solve failed: invalid reference projection")
            return

        # Optional refinement (improves accuracy)
        d = refine_depth_1d(
            self.V0, R_co, r_cam,
            self.fx, self.fy, self.cx, self.cy,
            w_obs, h_obs,
            d_init=d0, n_steps=40, step_scale=2.0
        )

        if d is None:
            d = d0

        # print("d",d)

        # object origin in camera coords (along the ray)
        t_co = d * r_cam  # p_obj_origin expressed in camera frame

        p_wo_from_cam = p_cam_w + R_wc @ t_co 

        #Consider shift of origin baked into the mesh vertices
        p_wo_from_cam_original = p_wo_from_cam + R_wo @ (-self.center)

        self.p_wo_from_cam = p_wo_from_cam_original

  

        # ---------------------------------------------------------
        # Convert to "camera position relative to object" etc.
        # ---------------------------------------------------------
        # Camera position in object frame:
        # p_oc = - R_oc * t_co  where R_oc = R_co^T
        R_oc = R_co.T
        p_oc = - R_oc @ t_co
    

        self.p_oc = p_oc
        # print(self.p_oc)

        # If you want the required camera position in world:
        p_wo = T_obj_w[:3, 3]
        p_wc_required = p_wo + R_wo @ p_oc

        rospy.loginfo_throttle(
            0.5,
            "YOLO bbox w,h=(%.1f, %.1f), ref w,h=(%.1f, %.1f) => d0=%.3f, d=%.3f",
            w_obs, h_obs, w_ref, h_ref, d0, d
        )
        rospy.loginfo_throttle(
            0.5,
            "Required camera position in WORLD: [%.3f, %.3f, %.3f] (assuming bbox center==object origin)",
            p_wc_required[0], p_wc_required[1], p_wc_required[2]
        )
        rospy.loginfo_throttle(
            0.5,
            "Camera position in OBJECT frame: [%.3f, %.3f, %.3f]",
            p_oc[0], p_oc[1], p_oc[2]
        )

                # ---------------------------------------------------------
        # Debug: draw YOLO bbox, projected bbox, and projected silhouette
        # ---------------------------------------------------------
        # img = np.zeros((H, W, 3), dtype=np.uint8)


        pix, _ = project_mesh_pinhole(self.V0, R_co, t_co, self.fx, self.fy, self.cx, self.cy)


        if pix.shape[0] >= 3:
            # Draw YOLO bbox in green
            # cv2.rectangle(img, (int(umin), int(vmin)), (int(umax), int(vmax)), (0, 255, 0), 2)

            # Projected bbox (red)
            umin_p, vmin_p, umax_p, vmax_p = bbox_from_pixels(pix)
            cv2.rectangle(img, (int(umin_p), int(vmin_p)), (int(umax_p), int(vmax_p)), (0, 0, 255), 2)
            cv2.putText(img,"d ="+f"{d:.2f}m", 
                (int(umin_p),int(vmax_p)+100), 
                cv2.FONT_HERSHEY_SIMPLEX, #font
                4,  #font scale
                (255,255,255), # font color
                2) #line thickness
            #Calculate position error
            e_offset = self.p_wo_from_cam - T_obj_w[:3,3]
            e = np.linalg.norm(e_offset)
            cv2.putText(img,"e ="+f"{e:.2f}m", 
                (int(umin_p),int(vmax_p)+200), 
                cv2.FONT_HERSHEY_SIMPLEX, #font
                4,  #font scale
                (255,255,255), # font color
                2) #line thickness

            # Mark center
            cv2.circle(img, (int(u0), int(v0)), 4, (255, 255, 255), -1)

            # -----------------------------
            # NEW: draw projected silhouette
            # -----------------------------
            # Keep only points that are reasonably near the image (avoid huge hulls if some verts go crazy)
            pad = 200  # pixels outside image allowed (tune)
            mask_in = (
                (pix[:, 0] > -pad) & (pix[:, 0] < self.width + pad) &
                (pix[:, 1] > -pad) & (pix[:, 1] < self.height + pad)
            )
            pix2 = pix[mask_in]

            if pix2.shape[0] >= 3:
                # Convex hull in pixel coords
                hull = ConvexHull(pix2)
                poly = pix2[hull.vertices]  # Mx2
                poly_i = np.round(poly).astype(np.int32)

                # Alpha-blended fill
                overlay = img.copy()
                cv2.fillPoly(overlay, [poly_i], (255, 0, 0))      # filled silhouette (blue)
                cv2.polylines(img, [poly_i], True, (255, 255, 0), 2)  # outline (cyan/yellow)

                alpha = 0.25  # transparency
                img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

        self.img_msg = self.cvbridge.cv2_to_imgmsg(img, encoding="passthrough")

        plane_origin_w = T_obj_w[:3, 3]
        plane_R_w = T_cam_w[:3, :3]
        # debug_visualize_scene(self.mesh, T_obj_w, T_cam_w, plane_origin_w=plane_origin_w, plane_R_w=plane_R_w)



if __name__ == "__main__":
    DaeProjector()
    rospy.spin()
