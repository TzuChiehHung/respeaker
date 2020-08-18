#!/usr/bin/env bash
# http://wiki.seeedstudio.com/ReSpeaker_2_Mics_Pi_HAT/


install_system_updates(){
    echo "updating system."
    sudo apt-get update
    sudo apt-get upgrade -y
}

install_dependencies(){
    echo "install dependencies."
    sudo apt-get install -y python-spidev
    sudo apt-get install -y python-rpi.gpio
}

install_respeaker(){
    echo "install respeaker."
    git clone https://github.com/respeaker/seeed-voicecard.git
    cd seeed-voicecard
    sudo ./install.sh
}

main(){
    install_system_updates
    install_dependencies
    install_respeaker
}

main "$@"
