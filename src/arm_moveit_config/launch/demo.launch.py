from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def scrub_empty(obj):
    if isinstance(obj, dict):
        cleaned = {}
        for k, v in obj.items():
            if isinstance(v, (list, tuple)) and len(v) == 0:
                continue  # Drop empty lists/tuples completely so launch_ros never converts [] -> ()
            sv = scrub_empty(v)
            if sv != {} and sv is not None:
                cleaned[k] = sv
        return cleaned
    elif isinstance(obj, (list, tuple)):
        return [scrub_empty(x) for x in obj]
    return obj

_orig_to_moveit_configs = MoveItConfigsBuilder.to_moveit_configs
def _patched_to_moveit_configs(self):
    mc = _orig_to_moveit_configs(self)
    for attr in dir(mc):
        if not attr.startswith("_"):
            val = getattr(mc, attr)
            if isinstance(val, dict):
                setattr(mc, attr, scrub_empty(val))
    return mc

MoveItConfigsBuilder.to_moveit_configs = _patched_to_moveit_configs

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("five_dof_arm", package_name="arm_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
