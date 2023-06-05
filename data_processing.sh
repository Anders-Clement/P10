#!/bin/bash

path="/media/jonas/BIG_USB/bag_files_testing_P10"

rate=20 #playbag speed of bag
clock_rate=$(($rate*100)) #pub rate of bag clock
snippet_length=600 #10min in sec
SECONDS=0
total_files=0
count=0

for d1 in $path/* ; do
    if [[ "$d1" != *"mapf"* ]] ; then
        continue
    fi
    for d2 in "$d1"/* ; do
        for d3 in "$d2"/* ; do
            total_files=$(($total_files+1))
        done
    done
done

for dir_q_type in $path/* ; do # static_queue or dyn_queue folder
    if [[ "$dir_q_type" != *"mapf"* ]] ; then
        continue
    fi

    for dir_rob_nr in "$dir_q_type"/* ; do # how many robots folder

        for ros_bag in "$dir_rob_nr"/*; do #the ros bag folder
            if [[ "$ros_bag" == *"csv"* ]] ; then
                continue
            fi
            count=$(($count+1))
            echo "$count of $total_files"

            for file in "$ros_bag"/* ; do
                
                if [[ "$file" != *"metadata.yaml"* ]] ; then
                    continue
                fi
                IFS=' ' #setting space as delimiter  
                while read -r line ; do 

                    if [[ "$line" == *"nanoseconds:"* ]] ; then
                        read -ra ADDR <<<"$line" #reading str as an array as tokens separated by IFS  
                        duration=$((${ADDR[1]}/1000000000)) # from nanosec to sec
                        break
                    fi

                done < "$file"
            done
            ##here we are in specific bag folder and have start_time and duration of the bag

            start_offset=$(($duration-$snippet_length)) #how many seconds into the bag should it start

            if (($start_offset < 0)) ; then
                echo "ERROR bag was not 10min long: $ros_bag"
                continue
            fi

            ros2 bag play --clock "$clock_rate" -r "$rate" --start-offset $start_offset --read-ahead-queue-size 500000 "$ros_bag"

            sleep 1s

            ros2 topic pub -1 /do_write_csv std_msgs/String "data: $ros_bag"

        done
    done
done

bash_duration=$SECONDS
echo "$(($bash_duration / 60)) minutes and $(($bash_duration % 60)) seconds elapsed."


