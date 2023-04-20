docker build -t vehicle_captain/cpp_receiver_client:0.1 -f Dockerfiles/Dockerfile.demo .
docker run -t -d --name cpp_receiver-demo --network=routing-bridge vehicle_captain/cpp_receiver_client:0.1
