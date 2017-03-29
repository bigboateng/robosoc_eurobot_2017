import high_level_secondary


# -- Secondary --

def get_list:
    get_rocket_cylinders("A", isYellow)
    drop_cylinders_in_moonbase("B", isYellow)
    get_stray_cylinder("A", isYellow)
    get_stray_cylinder("B", isYellow)
    drop_cylinders_in_moonbase("B", isYellow)
    get_stray_cylinder("C", isYellow)
    get_stray_cylinder("D", isYellow)
    get_stray_cylinder("E", isYellow)
    get_stray_cylinder("F", isYellow)
    drop_cylinders_in_moonbase("A", isYellow)
    rotate_cylinders_action("A")
    rotate_cylinders_action("B")
    rotate_cylinders_action("C")