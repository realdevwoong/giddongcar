# Duckietown lane-pose checkpoints

These two local checkpoints come from the Duckietown ETHZ machine-learning lane-following project and its `wickipedia/cnn_node` implementation:

- `distance_from_lane_center_statedict`: estimates lateral distance from the lane center.
- `heading_error_statedict`: estimates heading error relative to the lane.

The upstream implementation describes these as PyTorch `state_dict` files, not Ultralytics `.pt` models. They require the matching CNN architecture and image transforms from [`wickipedia/cnn_node`](https://github.com/wickipedia/cnn_node); they cannot be passed to `vision_drive --model` as-is. The training domain is Duckietown, so white tape on the Pinky course still needs observation-only validation.

The upstream repository does not include an explicit license file. Keep the checkpoint files local and do not redistribute them until the upstream licensing terms are confirmed. The `.gitignore` rule excludes the weights from commits.
