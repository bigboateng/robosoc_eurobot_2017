import high_level_primary

# -- Primary --

def init():
    pass


if __name__ == "__main__":
    # Initialise
    init()

    isYellow = True

    go_though_seasaw(isYellow);
	
	# If we want to steal balls from large crater on other team
	get_large_crater_balls(!isYellow)
	drop_off_balls()

    # High Level functions
    get_small_crater_balls(isYellow)
    drop_off_balls()
    get_large_crater_balls(isYellow)
    drop_off_balls()
