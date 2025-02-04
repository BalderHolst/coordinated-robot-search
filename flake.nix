{
    inputs = {
        flake-utils.url = "github:numtide/flake-utils";
        nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
        nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    };
    outputs = { flake-utils, nixpkgs, ... }@inputs:
        let
            container-name = "robot-search";
            dockerfile = "./Dockerfile";
        in
        flake-utils.lib.eachDefaultSystem (system:
            let
                commands = {
                    build = /*bash*/ ''
                        # Build if container does not exist
                        if [ ! "$(docker ps -aq -f name=$CONTAINER)" ]; then
                        echo "Container '$CONTAINER' does not exist. Building..."
                        docker build -t $CONTAINER -f ${dockerfile} .
                        fi
                    '';
                    start = /*bash*/ ''
                        build
                        # Start if container is stopped
                        if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER)" ]; then
                                echo "Starting container..."
                                docker start $CONTAINER > /dev/null
                        fi
                    '';
                    stop = /*bash*/  ''
                        # Stop if container is running
                        if [ "$(docker ps -aq -f status=running -f name=$CONTAINER)" ]; then
                                echo "Stopping container..."
                                docker stop $CONTAINER > /dev/null
                        else
                                echo "Container is not running"
                        fi
                    '';
                    run = /*bash*/ ''
                        # Make sure a command was provided
                        if [ -z "$1" ]; then
                                echo "No command provided"
                                exit 1
                        fi

                        build
                        start

                        cmd="$@"

                        docker exec -it \
                            --privileged \
                            --workdir="/root/ws" \
                            $CONTAINER bash -i -c "$cmd"
                    '';
                    enter = /*bash*/ ''
                            build
                            start

                            XAUTH=/tmp/.docker.xauth

                            [ "$(docker ps | grep $CONTAINER)" ] && {
                                    echo "Attaching to running container..."
                                    docker exec -it \
                                        --privileged \
                                        --workdir="/root/ws" \
                                        $CONTAINER bash
                                    exit 0
                            }

                            echo "Running container..."
                            docker run -it \
                                    --name $CONTAINER \
                                    --env="DISPLAY=$DISPLAY" \
                                    --env="QT_X11_NO_MITSHM=1" \
                                    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                                    --env="XAUTHORITY=$XAUTH" \
                                    --volume="$XAUTH:$XAUTH" \
                                    --net=host \
                                    --workdir="/root/ws" \
                                    --volume="$ROOT:/root/ws" \
                                    --privileged \
                                    $CONTAINER \
                                    bash
                    '';
                    delete = /*bash*/ ''
                        echo "Deleting container '$CONTAINER'..."
                        docker rm $CONTAINER > /dev/null
                    '';
                    help = /*bash*/ ''
                        echo -e "\nAvailable Commands:"
                        ${ lib.concatStringsSep "\n" (
                            lib.attrsets.mapAttrsToList (k: v: "echo -e '\t${k}'") commands
                        )}
                        echo ""
                    '';
                };
                pkgs = import nixpkgs {
                    inherit system;
                    overlays = [ inputs.nix-ros-overlay.overlays.default ];
                };
                lib = nixpkgs.lib;
            in {
        devShells.default = pkgs.mkShell {
            name = "Docker based development environment";

            packages = with pkgs; with pkgs.python3Packages; [
                git
                docker
                python3
                # rosPackages.humble.rclpy

            ] ++ lib.attrsets.mapAttrsToList
                (k: v: pkgs.writeShellScriptBin k v)
                commands;

            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (with pkgs; [
                wayland
                libGL
                libxkbcommon
            ]);

            shellHook = ''
                ${commands.help}

                export ROOT="$(pwd)"
                export CONTAINER="${container-name}"
            '';
        };
    });
    nixConfig = {
        extra-substituters = [ "https://ros.cachix.org" ];
        extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
    };
}
