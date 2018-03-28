def read_lat_lon(filename):
    with open(filename, 'r') as f:
        lat_lon = f.readline()
        lat_lon = [y for x in lat_lon.split(", ") for y in x.split(" ")]
        lat = float(lat_lon[1])
        lon = float(lat_lon[3])
        return lat, lon
