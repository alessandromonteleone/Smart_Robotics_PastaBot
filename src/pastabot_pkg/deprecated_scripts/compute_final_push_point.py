


def compute_side_points(push_point: list, direction: str, side_lengths: list):
    """
    push_point: middle point of bottom side in real-wold coords
    """
    x, y = push_point
    side_x, side_y = side_lengths

    if direction == "left":
        return (x + side_x/2, y + side_y/2)
    elif direction == "right":
        return (x + side_x/2, y - side_y/2)
    else:
        return push_point
    
def classification(distance):
    tol_hm = 0.02
    th_hm = 0.06

    tol_ms = 0.04
    th_ms = 0.18

    if distance < th_hm + tol_hm: #0,8
        box_class = 'h'
    elif th_hm + tol_hm <= distance < th_ms - tol_ms:   #0,8 - 0,14
        box_class = 'm'
    else:
        box_class = 's'
    print(box_class)
    return box_class

def distance_to_push_side(push_point: list, side_lengths: list, distance):
    
    box_class = classification(distance)

    if box_class == 'h':
        direction = 'front'
    elif box_class == 'm':
        direction = 'left'
    elif box_class == 's':
        direction = 'right'
    
    return compute_side_points(push_point, direction, side_lengths)

