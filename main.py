from controller import Controller
from PV import PV
from utils import calc_sensor_distance

if __name__ == '__main__':
    c = Controller(n_nodes=50)
    pv = PV()
    f_alive = open('alive.txt', 'w')
    f_iso = open('isolated.txt', 'w')
    f_dead = open('dead.txt', 'w')
    f_sleeping = open('sleeping.txt', 'w')

    print('Controller initialized')
  
    for i in range(100):
        print('Round: ' + str(i))
        # Epoch i
        #    Beacon
        for node in c.sensor_nodes:
            node.transmit(main=False)
        # print(c.node.E_rank_u_neighbors_beacon)
        #    Run ECCKN
        c.run_ECCKN(k=10)

        # print(controller.node.E_rank_u_neighbors)
        f_alive.write(str(i) + ',' + str(c.get_alive_nodes()) + '\n')
        f_iso.write(str(i) + ',' + str(c.get_isolated_nodes()) + '\n')
        f_dead.write(str(i) + ',' + str(c.get_dead_nodes()) + '\n')
        f_sleeping.write(
            str(i) + ',' + str(c.get_sleeping_nodes()) + '\n')

        # print('## Update topology ##')
        c.current_topology, c.shortest_path = c.update_topology_shortest_path()
        # print(c.shortest_path)

        # print('## Update sensor node targets ##')
        c.update_sensor_node_targets()

        #    Execution
        for node in c.sensor_nodes:
            node.transmit()

        c.node.update_energy(-1.0 * (115.85+1018.5+0.0001))

        #   Controller transmit beacon data to all nodes (for now)
        for node in c.sensor_nodes:
            c.node.transmit(controller_distance=calc_sensor_distance(c.node, node))

        # post-update properties
        c.update_sensor_properties()

        # PV energy
        c.update_energy(pv.get_E())

        # print('## Drawing.. ##')
        c.draw("t" + str(i) + ".png")

        # print('## saving E_rank..')
        c.save_erank(i)

        
    c.export_erank()
    print('DONE!')
