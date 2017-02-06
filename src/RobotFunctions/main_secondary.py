import high_level_secondary


# -- Secondary --

def init():
    pass


if __name__ == "__main__":
    # Initialise
    init()

    # For example
    get_rocket_cylinders("A", 4)
    drop_cylinders_in_moonbase("B")
    get_stray_cylinder("A")
    get_stray_cylinder("B")
    drop_cylinders_in_moonbase("B")
    get_stray_cylinder("C")
    get_stray_cylinder("D")
    get_stray_cylinder("E")
    get_stray_cylinder("F")
    drop_cylinders_in_moonbase("A")
    rotate_cylinders_action("A")
    rotate_cylinders_action("B")
    rotate_cylinders_action("C")