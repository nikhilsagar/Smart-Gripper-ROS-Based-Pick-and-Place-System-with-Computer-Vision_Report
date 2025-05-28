import cv2
import cv2.aruco as aruco

def generate_aruco(marker_id=0, marker_size=200, dictionary=aruco.DICT_4X4_50):
    """ Generates an ArUco marker, saves, and displays it """
    aruco_dict = aruco.Dictionary_get(dictionary)
    marker = aruco.drawMarker(aruco_dict, marker_id, marker_size)

    filename = f"aruco_marker_{marker_id}.png"
    cv2.imwrite(filename, marker)
    print(f"✅ ArUco Marker {marker_id} saved as {filename}")

    cv2.imshow(f"ArUco Marker {marker_id}", marker)
    cv2.waitKey(500)  # Show each marker for 0.5 sec
    cv2.destroyAllWindows()

def main():
    for marker_id in range(20):  # Generate 4 markers (ID 0 to 3)
        generate_aruco(marker_id)

if __name__ == "__main__":
    main()
