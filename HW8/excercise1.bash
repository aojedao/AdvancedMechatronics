# run this ina terminal.
for i in {1..5}; do
    libcamera-jpeg -o image_$i.jpg
    sleep 2
done
