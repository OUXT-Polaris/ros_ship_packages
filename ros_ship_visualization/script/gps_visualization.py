import commands
import yaml

class map_downloader:
    def __init__(self):
        self.left_lower_point = [135.524,34.822]
        self.right_upper_point = [135.526,34.823]
        self.osm_filename = "../data/map.osm"
        self.map_index_filename = "../data/map_index.yaml"
        self.map_id = 1
    def download(self):
        cmd = "wget -O "+ self.osm_filename +" http://overpass-api.de/api/map?bbox="+str(self.left_lower_point[0])+","+str(self.left_lower_point[1])+","+str(self.right_upper_point[0])+","+str(self.right_upper_point[1])
        #commands.getoutput(cmd)
        self.bbox_points = {"left_lower_point":{"latitude":self.left_lower_point[0],"longitude":self.left_lower_point[1]},"right_upper_point":{"latitude":self.right_upper_point[0],"longitude":self.right_upper_point[1]}}
    def query_same_parameters_map_ID(self):
        f = open(self.map_index_filename, "r+")
        map_data = yaml.load(f)
        f.close()
        for key in map_data.keys():
            if self.left_lower_point[0] == map_data[key]["bbox_points"]["left_lower_point"]["latitude"]:
                if self.left_lower_point[1] == map_data[key]["bbox_points"]["left_lower_point"]["longitude"]:
                    if self.right_upper_point[0] == map_data[key]["bbox_points"]["right_upper_point"]["latitude"]:
                        if self.right_upper_point[1] == map_data[key]["bbox_points"]["right_upper_point"]["longitude"]:
                            return map_data[key]["ID"]
        return None
    def save(self):
        map_data = {"ID="+str(self.map_id):{"bbox_points":self.bbox_points,"file":self.osm_filename,"ID":self.map_id}}
        #f = open(self.map_index_filename, "a")
        #print map_data
        #f.write(yaml.dump(map_data))
        #f.write(yaml.dump(map_data, default_flow_style=False))
        #f.close()

if __name__ == '__main__':
    downloader = map_downloader()
    downloader.download()
    if downloader.query_same_parameters_map_ID() != None:
        downloader.save()
