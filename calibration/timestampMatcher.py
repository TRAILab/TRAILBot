txtname = 'match.txt'
with open(txtname, "w") as output_file:
    
    # change the range of pointcloud file indices here
    for pt_file_index in range(331,500):
        # chanege the location of the pointcloud file here
        ptFile = f'points/points_{pt_file_index}.txt'
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
        for img_file_index in range(2101,3500):
            # chanege the location of the image file here
            imgFile = f'images/output_{img_file_index}.txt'
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
            if PC_time[0] == img_time[0] and abs(PC_time[1]-img_time[1]) < 5000000:
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