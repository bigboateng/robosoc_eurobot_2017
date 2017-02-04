import middle_level_secondary

# -- Secondary --

# Goes to and picks up n number of rockets from the specified rocket
def get_rocket_cylinders(rocket, n):
    go_to_rocket(rocket)
    for i in range(n):
        pick_up_rocket_cylinder()

# Goes to and picks up a stray cylinder
def get_stray_cylinder(cylinder):
    go_to_stray_cylinder(cylinder)
    pick_up_stray_cylinder()

# Goes to the specified moonbase and places the stored cylinders by going over the slot
def drop_cylinders_in_moonbase(moonbase):
    go_to_moonbase(moonbase)
    place_cylinders_in_slot()

# Goes to the side of the moonbase and rotates each cylinder
def rotate_cylinders_action(moonbase):
    go_to_moonbase_side(moonbase)
    rotate_cylinders_in_slot()