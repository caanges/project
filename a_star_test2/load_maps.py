import yaml
import os
import numpy as np
import cv2



def load_map(map_yaml):
    with open(map_yaml, "r") as open_file:
        map_info = yaml.safe_load(open_file)

    yaml_dir = os.path.dirname(map_yaml)
    image_path = os.path.join(yaml_dir, map_info["image"])

    map_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    obstacles =  map_info["occupied_thresh"] * 255
    map_img = np.where(map_img > obstacles, 0, 255).astype(np.uint8)
    
    scale_factor = 4  
    resized_img = cv2.resize(map_img, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)

    cv2.imshow("Map", resized_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    """
    padding_radius = 6
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (2 * padding_radius + 1, 2 * padding_radius + 1)
    )
    map_img = cv2.dilate(map_img, kernel, iterations=1)
    """
    free_space = map_info["free_thresh"] * 255
    origin = map_info["origin"]
    res = map_info["resolution"]
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return map_img, obstacles, free_space, origin, res


