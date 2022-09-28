main_folder=$1
for iter in {1..80}
do 
    seq_folder=$main_folder/$iter
    echo "sequence: $iter: $seq_folder"
    ./my_recons.sh $seq_folder
done



