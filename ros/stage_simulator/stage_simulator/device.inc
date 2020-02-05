define topurg ranger
(
  sensor(                         
    range [ 0.0  30.0 ]
    fov 270.25
    samples 1081
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define robot position
(
  size [0.35 0.35 0.25]
  origin [-0.05 0 0 0]
  gui_nose 1
  drive "diff"
  topurg(pose [ 0.050 0.000 0.000 0.000 ])
)
