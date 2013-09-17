from object_manager import ObjectManager
import outdoor_faraway_config as config


object_manager_ = ObjectManager()
object_manager_.create_objects(config, '20_1000_1000_random_homogeneous2_faraway.csv')

