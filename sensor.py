import numpy as np

from utils import State


class Sensor:
    def __init__(self, E_rank_u=100001, position=None, controller=False):
        """
        :param E_rank_u: Initial Energy
        :param position: Initial X,Y 2-tuple Position else randomized over (200,200)
        :param controller: Boolean, if sensor node is for controller
        """
        self.E_rank_u = E_rank_u
        self.state = State.INIT
        self.pos_x, self.pos_y = np.random.randint(
            1, 200, 2) if position is None else position
        self.E_elec = 1
        self.eps_amp = 1
        self.is_controller = controller

        # this should only be updated ONCE due to predetermined path supplied
        # by controller
        self.target_beacon = None
        self.target_beacon_distance = 0
        self.l_beacon = 1
        self.E_rank_u_neighbors_beacon = {}

        # This is continuously updated
        self.target_main = None
        self.target_main_distance = 0
        self.l_main = 1
        self.E_rank_u_neighbors_main = {}
        self.isolated = False

    def update_state(self, new_state):
        """
        :param new_state: must be a valid state determined by State class
        :return: Nothing if new_state is not a valid state
        """
        if State.is_valid(self.state, new_state):
            self.state = new_state
        else:
            return

    def update_properties(self):
        """
        Recalculate l_main based from E_rank_u_neighbors_main
        Controller sensor node is always awake, except for limited energy scenarios
        :return:
        """
        # TODO: except for limited energy scenarios
        if not self.is_controller:
            self.isolated = True if self.target_main is None else False
            self.l_main = len(self.E_rank_u_neighbors_main.items())

            # TODO: think about this.
            if self.E_rank_u <= 0:
                self.die()
        else:
            self.isolated = False
            self.state = State.AWAKE

    def get_position(self):
        """
        Return position
        :return:
        """
        return self.pos_x, self.pos_y

    def update_energy(self, energy):
        """
        Updates the E_rank_u
        :return:
        """
        self.E_rank_u += energy

    def wake_up(self):
        """
        Wake up Sensor node
        """
        self.update_state(State.AWAKE)

    def sleep(self):
        """
        Command Sensor node to go to sleep
        """
        self.update_state(State.SLEEP)

    def die(self):
        """
        Indicates Sensor node is dead
        :return:
        """
        self.update_state(State.DEAD)

    def set_target(self, target, target_distance, main=True):
        """
        Set target sensor node and distance for beacon or main data transmission
        :param target: Sensor node object
        :param target_distance: float type
        :param main: True if setting main target
        :return:
        """
        if self.state != State.DEAD and not self.is_controller:
            if main:
                self.target_main = target
                self.target_main_distance = target_distance
            else:
                self.target_beacon = target
                self.target_beacon_distance = target_distance

    def transmit(self, main=True):
        """
        Transmit beacon data, if possible
        Prevent controller sensor node from transmitting main data
        :param main: True if transmitting main data
        """
        if self.state != State.DEAD and not self.is_controller and self.target_main is not None:
            if main and self.state == State.AWAKE:
                E_usage = (
                    self.E_elec *
                    self.l_main +
                    self.eps_amp *
                    self.l_main *
                    self.target_main_distance ** 2)
                if E_usage <= self.E_rank_u:
                    self.update_energy(-1.0 * E_usage)
                    self.target_main.receive((self.get_name(), self.E_rank_u))
                else:
                    self.isolated = True

            else:
                E_usage = (
                    self.E_elec *
                    self.l_beacon +
                    self.eps_amp *
                    self.l_beacon *
                    self.target_beacon_distance ** 2)
                if E_usage <= self.E_rank_u:
                    self.update_energy(-1.0 * E_usage)
                    self.target_beacon.receive(
                        (self.get_name(), self.E_rank_u), main=False)

    def receive(self, neighbor_E_rank_u, main=True):
        """
        Receive data and E_rank_u from transmission, if possible
        :param neighbor_E_rank_u: 2-tuple containing sensor object and neighbor_E_rank_u
        :param main: True if receiving main data
        """
        if self.state != State.DEAD:
            if main and self.state == State.AWAKE:
                self.update_energy(-1.0 * self.E_elec * self.l_main)
                self.E_rank_u_neighbors_main[neighbor_E_rank_u[0]
                                             ] = neighbor_E_rank_u[1]
            else:
                self.update_energy(-1.0 * self.E_elec * self.l_beacon)
                self.E_rank_u_neighbors_beacon[neighbor_E_rank_u[0]
                                               ] = neighbor_E_rank_u[1]
        if self.is_controller:
            self.update_energy(200001)

    def get_name(self):
        """
        :return: String of Sensor ID based on position
        """
        return str((self.pos_x, self.pos_y))

    def is_asleep(self):
        """
        :return: Boolean if sensor is asleep
        """
        if self.state == State.SLEEP:
            return True
        else:
            return False

    def is_awake(self):
        """
        :return: Boolean if sensor is awake
        """
        if self.state == State.AWAKE:
            return True
        else:
            return False

    def is_dead(self):
        """
        :return: Boolean if sensor is asleep
        """
        if self.state == State.DEAD:
            return True
        else:
            return False

    def __repr__(self):
        return self.get_name()

    def __lt__(self, other):
        """
        IMPORTANT: this assumes that the controller is at (100, 100)
        this enables sorting and ultimately radial transmission of sensor nodes from farthest to nearest
        :param other: Sensor object
        :return:
        """
        if isinstance(other, Sensor):
            other_x, other_y = other.get_position()
            dist_self = ((self.pos_x - 100) ** 2 +
                         (self.pos_y - 100) ** 2) ** 0.5
            dist_other = ((other_x - 100) ** 2 + (other_y - 100) ** 2) ** 0.5
            if dist_self < dist_other:
                return True
            else:
                return False
