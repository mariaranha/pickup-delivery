# pickup-delivery
MC658 Project

Branch and Bound algorithm to solve Single Vehicle Pickup and Delivery Routing Problem

# Setup
Para rodar o projeto, mude o valor da variável global N do arquivo [ex_pickup_delivery.cpp](https://github.com/mariaranha/pickup-delivery/blob/319b73f1ce654c7b35d548d36974ba880af7e7e6/lab_mc658/src/ex_pickup_delivery.cpp#L27) para o número de nós da instância a ser rodada.

Ex.  
para a instância ./instances/pickup_delivery_10.dig  **N = 12** <br/>
para a instância ./instances/pickup_delivery_10.dig  **N = 22** <br/>

```
make all
./bin/ex_pickup_delivery.e  ./instances/pickup_delivery_5.dig 10 
```
Este projeto foi desenvolvido e executado em ambiente macOS
