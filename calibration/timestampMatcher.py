import numpy as np
import open3d as o3d
import shutil
import os


def matched_img_and_filtered_pcl(test_name, ptFile, imgFile, matches):
    x_min = -1
    x_max = 1
    y_min = 2 # 0.3, 1.25, 1.25 2(07)
    y_max = 5 # 3.2, 4.1, 4.1 5(07)
    z_min = -1
    z_max = 1.5
    points = []

    with open(ptFile, 'r') as pcl_file:
        for line in pcl_file.readlines():
            if line[0] == 'T': # This is the line in the file indicating Timestamp
                continue
            x, y, z = map(float, line.strip().split())
            # change the min max value for xy here
            if x > x_min and x < x_max and y > y_min and y < y_max and z > z_min and z < z_max:
                points.append([x, y, z])

    npArrayPoints = np.asarray(points)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(npArrayPoints)
    test_num = test_name[4:]
    o3d.io.write_point_cloud(f"{test_name}/matched{test_num}/pcl/match_{matches}.pcd", pcd)

    source_image = imgFile
    target_image = f'{test_name}/matched{test_num}/images/match_{matches}.jpg'

    print(source_image, target_image)
    shutil.copy(source_image, target_image)

def main():

    test_name = "test05" # test01, test02, test03, test07
    
    tolerance = 6e6  # nanosec # 9e6, 6e6, 6e6 , 6e6
    out_img_dir = os.path.join(test_name, "img-data")
    out_pcl_dir = os.path.join(test_name, "pcl-data")
    pcl_file_num = len(os.listdir(out_pcl_dir)) # 296, 314, 295, 306
    img_file_num = int(len(os.listdir(out_img_dir)) / 2) # 1299, 1360, 1256, 1411
    
    matches = 0
    txtname = f'{test_name}/match.txt'

    match_dir = os.path.join(test_name, f"matched{test_name[4:]}")
    match_image_dir = os.path.join(match_dir, "images")
    match_pcl_dir = os.path.join(match_dir, "pcl")

    if not os.path.exists(match_dir):
        os.makedirs(match_dir)
        os.makedirs(match_image_dir)
        os.makedirs(match_pcl_dir)
        

    with open(txtname, "w") as output_file:
        
        # change the range of pointcloud file indices here
        for pt_file_index in range(1, pcl_file_num+1):
            # chanege the location of the pointcloud file here
            ptFile = f'{test_name}/pcl-data/points_{pt_file_index}.txt'
            with open(ptFile, 'r') as f:
                timestamp = f.readline()
                PC_time = []
                current_num = ""
                for char in timestamp:
                    if char.isdigit():
                        current_num += char
                    elif current_num:
                        PC_time.append(int(current_num))
                        current_num = ""

                if current_num:
                    PC_time.append(int(current_num))

            matching_img = []

            # change the range of image file indices here
            for img_file_index in range(1, img_file_num+1):
                # chanege the location of the image file here
                imgFile = f'{test_name}/img-data/output_{img_file_index}.jpg'
                txtfile = f'{test_name}/img-data/output_{img_file_index}.txt'
                with open(txtfile, 'r') as f:
                    timestamp = f.readline()
                    img_time = []
                    current_num = ""
                for char in timestamp:
                    if char.isdigit():
                        current_num += char
                    elif current_num:
                        img_time.append(int(current_num))
                        current_num = ""

                if current_num:
                    img_time.append(int(current_num))

                # do the comparison
                if PC_time[0] == img_time[0] and (abs(PC_time[1]-img_time[1]) < tolerance):
                    # matching_img.append(img_file_index)
                    matching_img = img_file_index

                #clear img_time
                img_time = []

            #write down the matching img
            
            
            if  matching_img:
                # for img_num in matching_img:
                imgFile = f'{test_name}/img-data/output_{matching_img}.jpg'
                output_file.write(f"point cloud {pt_file_index}:")
                output_file.write(f" {matching_img}")
                output_file.write(f"\n")
                matches += 1

                matched_img_and_filtered_pcl(test_name, ptFile, imgFile, matches)

            # matching_img = 0
        print("Number of matches: ", matches, " out of: ", pcl_file_num)

        #clear PC_time
        PC_time = []
        f.close()

if __name__ == "__main__":
    main()