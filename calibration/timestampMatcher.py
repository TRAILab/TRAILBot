# txtname = '/home/trailbot/trail_ws/calibration_2025/ptp/match.txt'
txtname = '/home/trailbot/action4/20260227/outdoor_standard/match.txt'
import tqdm
with open(txtname, "w") as output_file:
    
    # change the range of pointcloud file indices here
    for pt_file_index in tqdm.tqdm(range(1,2082)):
        # chanege the location of the pointcloud file here
        ptFile = f'/home/trailbot/action4/20260227/outdoor_standard/lidar_data/points_{pt_file_index}.txt'
        with open(ptFile, 'r') as f:
            timestamp_line = f.readline().strip()  # e.g., "Timestamp: builtin_interfaces.msg.Time(sec=1749049790, nanosec=724755225)"
            
            # Extract numbers using split and replace
            timestamp_line = timestamp_line.replace("Timestamp: builtin_interfaces.msg.Time(", "").replace(")", "")
            time_parts = timestamp_line.split(", ")

            PC_time = []
            for part in time_parts:
                key, value = part.split("=")
                PC_time.append(int(value))

            # Now: PC_time = [1749049790, 724755225]


        matching_img = []

        # change the range of image 1 indices here
        for img_file_index in (range(202,3732)):
            # chanege the location of the image file here
            imgFile = f'/home/trailbot/action4/20260227/outdoor_standard/frame_{img_file_index}.txt'
            with open(imgFile, 'r') as f:
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
            
            #do the comparison

            if PC_time[0] == img_time[0] and abs(PC_time[1]-img_time[1]) < 1.0e7:
                matching_img.append(img_file_index)

            #clear img_time
            img_time = []

        #write down the matching img
        output_file.write(f"point cloud {pt_file_index}:")
        for img_num in matching_img:
            output_file.write(f" {img_num}")
        output_file.write(f"\n")

        #clear PC_time
        PC_time = []