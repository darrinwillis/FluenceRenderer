pid=""
inotifywait -r -m --exclude ".*\.(swp|tmp)" -e modify ../src |
while read -r filename event; do
    echo "$filename $event"
    if [ "$pid" != "" ]; then
        kill $pid
        pid=""
    fi
    make -j8
    ./pathtracer &
    pid="$!"
done
