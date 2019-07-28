import math


def calc_distance(start_point, end_point):
    distance = math.sqrt(
        (start_point[0] - end_point[0]) ** 2 +
        (start_point[1] - end_point[1]) ** 2 +
        (start_point[2] - end_point[2]) ** 2
    )
    return distance


def calc_speed(start_point, end_point, fps=10):
    time_delta = 1/fps
    distance = calc_distance(start_point, end_point)
    return distance / time_delta


def get_position(drone):
    return drone.matrix_world.to_translation()


def get_distance(drone1, drone2):
    point1 = get_position(drone1)
    point2 = get_position(drone2)

    return calc_distance(point1, point2)


def get_drone_properties(drone):
    return dict(filter(lambda x: x[0].lower().startswith("drone_"), drone.items()))


def add_bool_property(obj, name, description="bool property"):
    rna_ui = obj.get('_RNA_UI')
    if rna_ui is None:
        rna_ui = obj['_RNA_UI'] = {}
    obj[name] = 0

    rna_ui[name] = {"description": description,
                    "default": False,
                    "min": 0,
                    "max": 1,
                    "soft_min": 0,
                    "soft_max": 1}

    # def insert_prop_keyframe(obj, prop_path: str, value):
    # obj.keyframe_insert(data_path='["prop"]')