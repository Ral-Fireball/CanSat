lat = []
long = []
time = []
co = []
nh3 = []
no2 = []
o3 = []
pm = []
temp = []
press = []
alt = []

while True:
    data_string = input("Enter a data string with 5 data bits separated by commas, or type 'quit' to exit: ")

    if data_string == "quit":
        break

    data_bits = data_string.split(",")

    data_bits = [float(bit) for bit in data_bits]

    lat.append(data_bits[0])
    long.append(data_bits[1])
    time.append(data_bits[2])
    co.append(data_bits[3])
    nh3.append(data_bits[4])
    no2.append(data_bits[5])
    o3.append(data_bits[6])
    pm.append(data_bits[7])
    temp.append(data_bits[8])
    press.append(data_bits[9])
    alt.append(data_bits[10])

    print("Latitude: ", lat)
    print("Longitude: ", long)
    print("Time:", time)
    print("CO: ", co)
    print("NH3: ", nh3)
    #hier komt matplotlib ipv dit printen, we moeten wel de main loop nog stoppen.



