# micromouse_stm32
A micromouse maze solving bot based on stm32 microcontroller
def display_engine_data(data):
    rpm = data['rpm'].value
    speed = data['speed'].value
    coolant_temp = data['coolant_temp'].value
    oil_pressure = data['oil_pressure'].value
    throttle_position = data['throttle_position'].value

    table = [
        ['RPM', rpm, 'rpm', '⚠️' if rpm > 6000 and speed < 50 else ''],
        ['Speed', speed, 'km/h', ''],
        ['Coolant Temp', coolant_temp, '°C', '⚠️' if coolant_temp > 105 else ''],
        ['Oil Pressure', oil_pressure, 'bar', '⚠️' if oil_pressure < 1.5 or oil_pressure > 4.5 else ''],
        ['Throttle Pos', throttle_position, '%', '⚠️' if throttle_position > 90 else ''],
    ]

    print("\nEngine Data:")
    print(tabulate(table, headers=["Parameter", "Value", "Unit", "Status"], tablefmt="fancy_grid"))
