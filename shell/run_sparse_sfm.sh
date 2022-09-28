# Run colmap from terminal, sparse sfm without optimizing the intrinsics. 
# Before you run this script, please intall colmap first.
# You need to specify the intrinsic parameters in "--ImageReader.camera_params"
# Example "run_sparse_sfm.sh /your/save/folder/path"
mkdir $1/models
./colmap feature_extractor --image_path  $1/img --database_path $1/models/db.db --ImageReader.camera_model PINHOLE --ImageReader.camera_params 897.4566,896.7992,635.4040,375.3149
colmap exhaustive_matcher --database_path $1/models/db.db
colmap mapper --image_path $1/img  --database_path $1/models/db.db --output_path $1/models --Mapper.ba_refine_focal_length 0 --Mapper.ba_refine_extra_params 0
./colmap model_converter --input_path $1/models/0  --output_path $1/models --output_type TXT
