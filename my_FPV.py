import fly_up_lib

copter = fly_up_lib.Copter()

copter.connect()
copter.arm()
copter.update()
copter.fly_up(7)
copter.turn(0)
copter.fly_forward(100)
copter.turn(90)
copter.fly_forward(100)
copter.turn(180)
copter.fly_forward(100)
copter.turn(270)
copter.fly_forward(100)

