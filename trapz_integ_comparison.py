# comparing processing power against the arduino using the same program


def main():
    x = 0
    dx_ms = 0.1
    dx = dx_ms / 1000
    y_prev = 0
    area1 = 0
    area6 = 0
    while True:
        # get y value
        y_now = pretend_sensor(x)

        # add to running total
        new_area = trapz_integ(y_prev, y_now, dx)
        y_prev = y_now

        area6 += new_area
        if x <= 1:
            area1 += new_area

        # increase the x placement
        x = x + dx

        # loop conditions
        if x > 6:
            break

    print("area1: ", area1)
    print("area6: ", area6)


def pretend_sensor(x):
    y = -(pow(0.5 * x - 1, 2)) + 10

    return y


def trapz_integ(y_prev, y_now, dx):
    this_area = (0.5 * dx) * (y_prev + y_now)

    return this_area


main()
