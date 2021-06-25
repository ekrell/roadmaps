# Roadmap experiments for GSCS 6321 - Geospatial Data Structures

# Which experiments to run
BUILD_PRM=false
SOLVE_PRM=true

# Paths
outdir=experiments/GSCS6321/

# Environment: full-sized raster
full_raster="full.tif"
ntrials=5

# Tasks: (start, goal pairs)


# PRM hyperparameters
num_samples_arr=(100 200 300 400 500 600 700 800 900 1000)
neighborhood_dist_arr=(100 200 300 400 500)


# Build PRMs 
if [ $BUILD_PRM == true ]; then 
    echo "hi"
    exit 0 
    start=1
    for n in ${num_samples_arr[@]}; do
        for d in ${neighborhood_dist_arr[@]}; do
            for (( t=$start; t<=$ntrials; t++ )); do
                python prm.py \
                    --build_roadmap \
                    --raster_file    $full_raster \
                    --roadmap_file   $outdir""/roadmap_n$n""_d$d""__$t"".pickle \
                    --map_plot       $outdir""/roadmap_n$n""_d$d""__$t"".png \
                    --num_samples $n \
                    --neighborhood_distance $d \
                    > $outdir""/roadmap_n$n""_d$d""__$t"".out
            done
        done
    done
fi 

# Solve with PRMs
if [ $SOLVE_PRM == true ]; then
    echo "solve"
fi
