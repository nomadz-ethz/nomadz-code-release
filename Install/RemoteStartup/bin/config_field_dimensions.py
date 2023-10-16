import argparse
import json
import os

DEFAULT_FIELD_DIMENSIONS_CFG_PATH = "/home/nao/Config/Locations/Remote/"

FIELD_DIMENSIONS_2020_CFG_TEMPLATE = """
xPosOpponentFieldBorder = {xPosOpponentFieldBorder};
xPosOpponentGoal = {xPosOpponentGoal};
xPosOpponentGoalPost = {xPosOpponentGoalPost};
xPosOpponentGroundline = {xPosOpponentGroundline};
xPosOpponentPenaltyArea = {xPosOpponentPenaltyArea};
xPosOpponentGoalBox = {xPosOpponentGoalBox};
xPosOpponentDropInLine = 0;
xPosOpponentPenaltyMark = {xPosOpponentPenaltyMark};
xPosPenaltyStrikerStartPosition = 0;
xPosHalfWayLine = {xPosHalfWayLine};
xPosOwnPenaltyMark = -xPosOpponentPenaltyMark;
xPosOwnDropInLine = -xPosOpponentDropInLine;
xPosOwnPenaltyArea = -xPosOpponentPenaltyArea;
xPosOwnGoalBox = -xPosOpponentGoalBox;
xPosOwnGroundline = -xPosOpponentGroundline;
xPosOwnGoalPost = -xPosOpponentGoalPost;
xPosOwnGoal = -xPosOpponentGoal;
xPosOwnFieldBorder = -xPosOpponentFieldBorder;

yPosLeftFieldBorder = {yPosLeftFieldBorder};
yPosLeftSideline = {yPosLeftSideline};
yPosLeftDropInLine = 0;
yPosLeftPenaltyArea = {yPosLeftPenaltyArea};
yPosLeftGoalBox = {yPosLeftGoalBox};
yPosLeftGoal = {yPosLeftGoal};
yPosCenterGoal = {yPosCenterGoal};
yPosRightGoal = -yPosLeftGoal;
yPosRightPenaltyArea = -yPosLeftPenaltyArea;
yPosRightGoalBox = -yPosLeftGoalBox;
yPosRightDropInLine = -yPosLeftDropInLine;
yPosRightSideline = -yPosLeftSideline;
yPosRightFieldBorder = -yPosLeftFieldBorder;

fieldLinesWidth = {fieldLinesWidth};
centerCircleRadius = {centerCircleRadius};
goalPostRadius = {goalPostRadius};
goalHeight = {goalHeight};
ballRadius = 33;
ballFriction = -0.5; // Used in: acceleration = ballFriction * velocity, dimension is 1/s

carpetBorder = [
  {{
    from = {{x = xPosOpponentFieldBorder; y = yPosRightFieldBorder;}};
    to   = {{x = xPosOpponentFieldBorder; y = yPosLeftFieldBorder;}};
  }},{{
    from = {{x = xPosOpponentFieldBorder; y = yPosLeftFieldBorder;}};
    to   = {{x = xPosOwnFieldBorder;      y = yPosLeftFieldBorder;}};
  }},{{
    from = {{x = xPosOwnFieldBorder;      y = yPosLeftFieldBorder;}};
    to   = {{x = xPosOwnFieldBorder;      y = yPosRightFieldBorder;}};
  }},{{
    from = {{x = xPosOwnFieldBorder;      y = yPosRightFieldBorder;}};
    to   = {{x = xPosOpponentFieldBorder; y = yPosRightFieldBorder;}};
  }}
];

fieldBorder = [
  {{
    from = {{x = xPosOpponentGroundline; y = yPosRightSideline;}};
    to   = {{x = xPosOpponentGroundline; y = yPosLeftSideline;}};
  }},{{
    from = {{x = xPosOpponentGroundline; y = yPosLeftSideline;}};
    to   = {{x = xPosOwnGroundline;      y = yPosLeftSideline;}};
  }},{{
    from = {{x = xPosOwnGroundline;      y = yPosLeftSideline;}};
    to   = {{x = xPosOwnGroundline;      y = yPosRightSideline;}};
  }},{{
    from = {{x = xPosOwnGroundline;      y = yPosRightSideline;}};
    to   = {{x = xPosOpponentGroundline; y = yPosRightSideline;}};
  }}
];

fieldLines = [
  // field border lines
  {{
    from = {{x = xPosOpponentGroundline; y = yPosRightSideline;}};
    to   = {{x = xPosOpponentGroundline; y = yPosLeftSideline;}};
  }},{{
    from = {{x = xPosOpponentGroundline; y = yPosLeftSideline;}};
    to   = {{x = xPosOwnGroundline;      y = yPosLeftSideline;}};
  }},{{
    from = {{x = xPosOwnGroundline;      y = yPosLeftSideline;}};
    to   = {{x = xPosOwnGroundline;      y = yPosRightSideline;}};
  }},{{
    from = {{x = xPosOwnGroundline;      y = yPosRightSideline;}};
    to   = {{x = xPosOpponentGroundline; y = yPosRightSideline;}};
  }},

// center line
  {{
    from = {{x = xPosHalfWayLine; y = yPosLeftSideline;}};
    to   = {{x = xPosHalfWayLine; y = yPosRightSideline;}};
  }},

// penalty areas
  {{
    from = {{x = xPosOwnGroundline;  y = yPosLeftPenaltyArea;}};
    to   = {{x = xPosOwnPenaltyArea; y = yPosLeftPenaltyArea;}};
  }},{{
    from = {{x = xPosOwnPenaltyArea; y = yPosLeftPenaltyArea;}};
    to   = {{x = xPosOwnPenaltyArea; y = yPosRightPenaltyArea;}};
  }},{{
    from = {{x = xPosOwnPenaltyArea; y = yPosRightPenaltyArea;}};
    to   = {{x = xPosOwnGroundline;  y = yPosRightPenaltyArea;}};
  }},

  {{
    from = {{x = xPosOpponentGroundline;  y = yPosLeftPenaltyArea;}};
    to   = {{x = xPosOpponentPenaltyArea; y = yPosLeftPenaltyArea;}};
  }},{{
    from = {{x = xPosOpponentPenaltyArea; y = yPosLeftPenaltyArea;}};
    to   = {{x = xPosOpponentPenaltyArea; y = yPosRightPenaltyArea;}};
  }},{{
    from = {{x = xPosOpponentPenaltyArea; y = yPosRightPenaltyArea;}};
    to   = {{x = xPosOpponentGroundline;  y = yPosRightPenaltyArea;}};
  }},

// goal boxes
  {{
    from = {{x = xPosOwnGroundline; y = yPosLeftGoalBox;}};
    to   = {{x = xPosOwnGoalBox;    y = yPosLeftGoalBox;}};
  }},{{
    from = {{x = xPosOwnGoalBox;    y = yPosLeftGoalBox;}};
    to   = {{x = xPosOwnGoalBox;    y = yPosRightGoalBox;}};
  }},{{
    from = {{x = xPosOwnGoalBox;    y = yPosRightGoalBox;}};
    to   = {{x = xPosOwnGroundline; y = yPosRightGoalBox;}};
  }},

  {{
    from = {{x = xPosOpponentGroundline; y = yPosLeftGoalBox;}};
    to   = {{x = xPosOpponentGoalBox;    y = yPosLeftGoalBox;}};
  }},{{
    from = {{x = xPosOpponentGoalBox;    y = yPosLeftGoalBox;}};
    to   = {{x = xPosOpponentGoalBox;    y = yPosRightGoalBox;}};
  }},{{
    from = {{x = xPosOpponentGoalBox;    y = yPosRightGoalBox;}};
    to   = {{x = xPosOpponentGroundline; y = yPosRightGoalBox;}};
  }},

// penalty and center marks
  {{
    from = {{x = {xStartOpponentPenaltyMark}; y = 0;}};
    to   = {{x = {xEndOpponentPenaltyMark}; y = 0;}};
  }},{{
    from = {{x = xPosOpponentPenaltyMark; y = -fieldLinesWidth;}};
    to   = {{x = xPosOpponentPenaltyMark; y =  fieldLinesWidth;}};
  }},

  {{
    from = {{x = {xStartOwnPenaltyMark}; y = 0;}};
    to   = {{x = {xEndOwnPenaltyMark}; y = 0;}};
  }},{{
    from = {{x = xPosOwnPenaltyMark; y = -fieldLinesWidth;}};
    to   = {{x = xPosOwnPenaltyMark; y =  fieldLinesWidth;}};
  }},

  {{
    from = {{x = -fieldLinesWidth; y = 0;}};
    to   = {{x =  fieldLinesWidth; y = 0;}};
  }}
];

centerCircle = {{
  center = {{x = xPosHalfWayLine; y = 0;}};
  radius = centerCircleRadius;
  numOfSegments = 16;
}};

xCorner = [
  {{x = xPosHalfWayLine; y = centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = -centerCircleRadius;}}
];

tCorner0 = [
  {{x = xPosHalfWayLine; y = centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = -centerCircleRadius;}},
  {{x = xPosOwnGroundline; y = yPosLeftPenaltyArea;}},
  {{x = xPosOwnGroundline; y = yPosRightPenaltyArea;}},
  {{x = xPosOwnGroundline; y = yPosLeftGoalBox;}},
  {{x = xPosOwnGroundline; y = yPosRightGoalBox;}}
];

tCorner90 = [
  {{x = xPosHalfWayLine; y = centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = -centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = yPosRightSideline;}}
];

tCorner180 = [
  {{x = xPosHalfWayLine; y = centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = -centerCircleRadius;}},
  {{x = xPosOpponentGroundline; y = yPosLeftPenaltyArea;}},
  {{x = xPosOpponentGroundline; y = yPosRightPenaltyArea;}},
  {{x = xPosOpponentGroundline; y = yPosLeftGoalBox;}},
  {{x = xPosOpponentGroundline; y = yPosRightGoalBox;}}
];

tCorner270 = [
  {{x = xPosHalfWayLine; y = centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = -centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = yPosLeftSideline;}}
];

lCorner0 = [
  {{x = xPosHalfWayLine; y = centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = -centerCircleRadius;}},
  {{x = xPosOwnGroundline; y = yPosLeftPenaltyArea;}},
  {{x = xPosOwnGroundline; y = yPosRightPenaltyArea;}},
  {{x = xPosOwnGroundline; y = yPosLeftGoalBox;}},
  {{x = xPosOwnGroundline; y = yPosRightGoalBox;}},
  {{x = xPosHalfWayLine; y = yPosRightSideline;}},
  {{x = xPosOwnGroundline; y = yPosRightSideline;}},
  {{x = xPosOpponentPenaltyArea; y = yPosRightPenaltyArea;}},
  {{x = xPosOpponentGoalBox; y = yPosRightGoalBox;}}
];

lCorner90 = [
  {{x = xPosHalfWayLine; y = centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = -centerCircleRadius;}},
  {{x = xPosOpponentGroundline; y = yPosLeftPenaltyArea;}},
  {{x = xPosOpponentGroundline; y = yPosRightPenaltyArea;}},
  {{x = xPosOpponentGroundline; y = yPosLeftGoalBox;}},
  {{x = xPosOpponentGroundline; y = yPosRightGoalBox;}},
  {{x = xPosHalfWayLine; y = yPosRightSideline;}},
  {{x = xPosOpponentGroundline; y = yPosRightSideline;}},
  {{x = xPosOwnPenaltyArea; y = yPosRightPenaltyArea;}},
  {{x = xPosOwnGoalBox; y = yPosRightGoalBox;}}
];

lCorner180 = [
  {{x = xPosHalfWayLine; y = centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = -centerCircleRadius;}},
  {{x = xPosOpponentGroundline; y = yPosLeftPenaltyArea;}},
  {{x = xPosOpponentGroundline; y = yPosRightPenaltyArea;}},
  {{x = xPosOpponentGroundline; y = yPosLeftGoalBox;}},
  {{x = xPosOpponentGroundline; y = yPosRightGoalBox;}},
  {{x = xPosHalfWayLine; y = yPosLeftSideline;}},
  {{x = xPosOpponentGroundline; y = yPosLeftSideline;}},
  {{x = xPosOwnPenaltyArea; y = yPosLeftPenaltyArea;}},
  {{x = xPosOwnGoalBox; y = yPosLeftGoalBox;}}
];

lCorner270 = [
  {{x = xPosHalfWayLine; y = centerCircleRadius;}},
  {{x = xPosHalfWayLine; y = -centerCircleRadius;}},
  {{x = xPosOwnGroundline; y = yPosLeftPenaltyArea;}},
  {{x = xPosOwnGroundline; y = yPosRightPenaltyArea;}},
  {{x = xPosOwnGroundline; y = yPosLeftGoalBox;}},
  {{x = xPosOwnGroundline; y = yPosRightGoalBox;}},
  {{x = xPosHalfWayLine; y = yPosLeftSideline;}},
  {{x = xPosOwnGroundline; y = yPosLeftSideline;}},
  {{x = xPosOpponentPenaltyArea; y = yPosLeftPenaltyArea;}},
  {{x = xPosOpponentGoalBox; y = yPosLeftGoalBox;}}
];
"""


def generate_field_dimensions_cfg(field_dimensions):
    field_params = field_dimensions["field"]
    goal_params = field_dimensions["goal"]
    field_half_length = field_params["length"] / 2
    field_half_width = field_params["width"] / 2
    lines_width = 50
    penalty_mark_dist_from_center = (
        field_half_length - field_params["penaltyCrossDistance"]
    )
    if "goalBoxAreaLength" in field_params:
        goal_box_x_from_center = field_half_length - field_params["goalBoxAreaLength"]
        goal_box_left_y_from_center = field_params["goalBoxAreaWidth"] / 2
    else:
        goal_box_x_from_center = 0
        goal_box_left_y_from_center = 0
    params_mapping = {
        "xPosOpponentFieldBorder": int(
            field_half_length + field_params["borderStripWidth"]
        ),
        "xPosOpponentGoal": int(field_half_length + goal_params["depth"]),
        "xPosOpponentGoalPost": int(field_half_length + lines_width / 2),
        "xPosOpponentGroundline": int(field_half_length),
        "xPosOpponentPenaltyArea": int(
            field_half_length - field_params["penaltyAreaLength"]
        ),
        "xPosOpponentGoalBox": int(goal_box_x_from_center),
        "xPosOpponentPenaltyMark": int(penalty_mark_dist_from_center),
        "xPosHalfWayLine": 0,
        "yPosLeftFieldBorder": int(field_half_width + field_params["borderStripWidth"]),
        "yPosLeftSideline": int(field_half_width),
        "yPosLeftPenaltyArea": int(field_params["penaltyAreaWidth"] / 2),
        "yPosLeftGoalBox": int(goal_box_left_y_from_center),
        "yPosLeftGoal": int(goal_params["innerWidth"] / 2),
        "yPosCenterGoal": 0,
        "fieldLinesWidth": int(lines_width),
        "centerCircleRadius": int(field_params["centerCircleDiameter"] / 2),
        "goalPostRadius": int(goal_params["postDiameter"] / 2),
        "goalHeight": int(goal_params["height"]),
        "xStartOpponentPenaltyMark": int(penalty_mark_dist_from_center - lines_width),
        "xEndOpponentPenaltyMark": int(penalty_mark_dist_from_center + lines_width),
        "xStartOwnPenaltyMark": int(-penalty_mark_dist_from_center + lines_width),
        "xEndOwnPenaltyMark": int(-penalty_mark_dist_from_center - lines_width),
    }

    return FIELD_DIMENSIONS_2020_CFG_TEMPLATE.format(**params_mapping)


def config_field_dimensions(
    field_dimensions_json_path,
    output_dir_path,
):
    assert os.path.exists(field_dimensions_json_path)
    assert os.path.exists(output_dir_path)
    field_dimensions_cfg_path = os.path.join(output_dir_path, "fieldDimensions.cfg")

    # Load the field_dimensions.json
    with open(field_dimensions_json_path, "r") as j:
        field_dimensions = json.load(j)

    for params_group in ["field", "goal"]:
        params = field_dimensions[params_group]
        for key in params:
            params[key] = 1000 * params[key]

    # Generate the field dimensions cfg
    field_dimensions_cfg = generate_field_dimensions_cfg(
        field_dimensions=field_dimensions
    )

    # Save it to the predefined location
    with open(field_dimensions_cfg_path, "w") as f:
        f.write(field_dimensions_cfg)


def parse_args():
    description = """
  Generates a fieldDimensions.cfg for the NomadZ framework
  from a field_dimensions.json
  """

    parser = argparse.ArgumentParser(description=description)

    parser.add_argument(
        "field_dimensions_json_path",
        type=str,
        help="Absolute path to the field_dimensions.json",
    )

    parser.add_argument(
        "-o",
        "--output-dir-path",
        type=str,
        default=DEFAULT_FIELD_DIMENSIONS_CFG_PATH,
        help="Absolute path to the directory where the fieldDimension.cfg should be saved",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    config_field_dimensions(
        field_dimensions_json_path=args.field_dimensions_json_path,
        output_dir_path=args.output_dir_path,
    )
