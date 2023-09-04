import fly_lib

copter = fly_lib.Copter()

copter.connect()
copter.arm()
copter.fly_up(20)