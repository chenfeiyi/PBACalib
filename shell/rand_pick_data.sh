#  randomly pick N number of images and pont cloud from raw data folders
# example "./rand_pick_data.sh /your/data/folder"
folder=$1
pose_num=7
pcd_folder=$folder/pcd
img_folder=$folder/img_un
pcd_files=$(ls $pcd_folder)
img_files=$(ls $img_folder)
i=1
for ele in ${img_files} 
do  
    # echo "ele $ele, i $i"
    img_files[$i]=$ele
    i=$(($i+1))
done

i=1
for ele in ${pcd_files} 
do  
    # echo "ele $ele, i $i"
    pcd_files[$i]=$ele
    i=$(($i+1))
done

for iter in {1..50}
do
    echo "---------iter $iter------------"
    save_path=$folder/../pose$pose_num/$iter
    mkdir -p $save_path/img
    mkdir -p $save_path/pcd
    num=$(shuf -i 1-12)
    i=1
    for ele in ${num} 
    do  
        echo "ele $ele, i $i"
        num[$i]=$ele
        i=$(($i+1))
    done
    for idx in $(seq 1 $pose_num)
    do 
        file_idx=${num["$idx"]}
        file=${img_files["$file_idx"]}
        save_path_img=$save_path/img/$file
        source_file=$img_folder/$file
        echo "idx $idx, file idx: $file_idx, save_path_img: $save_path_img, source_file: $source_file"
        cp $source_file $save_path_img
    done
    for idx in $(seq 1 $pose_num)
    do 
        file_idx=${num["$idx"]}
        file=${pcd_files["$file_idx"]}
        save_path_pcd=$save_path/pcd/$file
        source_file=$pcd_folder/$file
        echo "idx $idx, file idx: $file_idx, save_path_pcd: $save_path_pcd, source_file: $source_file"
        cp $source_file $save_path_pcd
    done
done


