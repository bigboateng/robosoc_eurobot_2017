import middle_level_primary


# -- Primary --

# Goes to the specified crater and picks up balls
def get_balls(crater):
    go_to_crater(crater)
    pick_up_balls(crater)

# Goes to the cargo bay in the starting area to place the balls in the net 
#   and then go back through the seasaw
def drop_off_balls():
    go_to_seasaw()
    put_balls_in_net()