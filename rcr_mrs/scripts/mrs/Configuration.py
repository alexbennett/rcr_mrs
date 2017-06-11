import yaml

class Configuration:
    def __init__(self, file_name):
        # Open file
        self._config_file = open(file_name, 'r')

        # Parse YAML
        self._config_yaml = yaml.load(self._config_file)

        # Close file
        self._config_file.close()

    def get_target_location(self):
        return (self._config_yaml['target_gps']['lat'], self._config_yaml['target_gps']['lon'], self._config_yaml['target_gps']['alt'])

    def get_landing_location(self):
        return (self._config_yaml['landing_gps']['lat'], self._config_yaml['landing_gps']['lon'], self._config_yaml['landing_gps']['alt'])

    def get_ground_speed_mult(self):
        return self._config_yaml['ground_speed_multiplier']

    def get_tds_loiter_time(self):
        return self._config_yaml['tds_loiter_time']

    def get_landing_loiter_time(self):
        return self._config_yaml['landing_loiter_time']
