"""UFLD 白線ベースの周回マップ作成 + QP レーシングライン走行 (ROS 非依存のコアロジック)."""

from .geometry import CameraModel, LineFit, fit_line, ground_to_image, project_to_ground
from .line_tracker import LineTracker, LineTrackerParams, TrackedLines
from .boundary_recorder import BoundaryRecorder, RecorderParams, spacing_for_curvature
from .course_map import CourseMap, LapDetectorParams, build_course_map
from .raceline_qp import Raceline, RacelineParams, optimize_raceline
from .path_tracker import RacelineFollower, TrackerParams
from .navigator import LaneNavigator, NavigatorParams, MAPPING, OPTIMIZING, RACING, STOPPED
from .cone_avoidance import (ConeRecorder, ReactiveAvoider, cone_landmark_correction,
                             deflect_raceline_around_cones)
