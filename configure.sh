ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/scripts/utils/print_color.sh

if [ -f /proc/device-tree/model ] && [[ "$(cat /proc/device-tree/model)" =~ "Jetson Orin Nano" ]]; then
    print_info "Detected Jetson Orin Nano"
    cp docker/compose_files/docker-compose-jon.yml.example docker-compose.yml
    print_info "Copying docker-compose-jon.yml.example to docker-compose.yml"
else
    print_info "Did not detect Jetson Orin Nano"
    cp docker/compose_files/docker-compose-pc.yml.example docker-compose.yml
    print_info "Copying docker-compose-pc.yml.example to docker-compose.yml"
fi