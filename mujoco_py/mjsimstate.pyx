
class MjSimState(namedtuple('SimStateBase', 'time qpos qvel act udd_state')):
    """Represents a snapshot of the simulator's state.

    This includes time, qpos, qvel, act, and udd_state.
    """
    __slots__ = ()

    # need to implement this because numpy doesn't support == on arrays
    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return NotImplemented

        if set(self.udd_state.keys()) != set(other.udd_state.keys()):
            return False

        for k in self.udd_state.keys():
            if isinstance(self.udd_state[k], Number) and self.udd_state[k] != other.udd_state[k]:
                return False
            elif not np.array_equal(self.udd_state[k], other.udd_state[k]):
                return False

        return (self.time == other.time and
                np.array_equal(self.qpos, other.qpos) and
                np.array_equal(self.qvel, other.qvel) and
                np.array_equal(self.act, other.act))

    def __ne__(self, other):
        return not self.__eq__(other)

    def flatten(self):
        """ Flattens a state into a numpy array of numbers."""
        if self.act is None:
            act = np.empty(0)
        else:
            act = self.act
        state_tuple = ([self.time], self.qpos, self.qvel, act,
                       MjSimState._flatten_dict(self.udd_state))
        return np.concatenate(state_tuple)

    @staticmethod
    def _flatten_dict(d):
        a = []
        for k in sorted(d.keys()):
            v = d[k]
            if isinstance(v, Number):
                a.extend([v])
            else:
                a.extend(v.ravel())

        return np.array(a)

    @staticmethod
    def from_flattened(array, sim):
        idx_time = 0
        idx_qpos = idx_time + 1
        idx_qvel = idx_qpos + sim.model.nq
        idx_act = idx_qvel + sim.model.nv
        idx_udd = idx_act + sim.model.na

        time = array[idx_time]
        qpos = array[idx_qpos:idx_qpos + sim.model.nq]
        qvel = array[idx_qvel:idx_qvel + sim.model.nv]
        if sim.model.na == 0:
            act = None
        else:
            act = array[idx_act:idx_act + sim.model.na]
        flat_udd_state = array[idx_udd:]
        udd_state = MjSimState._unflatten_dict(flat_udd_state, sim.udd_state)

        return MjSimState(time, qpos, qvel, act, udd_state)

    @staticmethod
    def _unflatten_dict(a, schema_example):
        d = {}
        idx = 0
        for k in sorted(schema_example.keys()):
            schema_val = schema_example[k]
            if isinstance(schema_val, Number):
                val = a[idx]
                idx += 1
                d[k] = val
            else:
                assert isinstance(schema_val, np.ndarray)
                val_array = a[idx:idx+schema_val.size]
                idx += schema_val.size
                val = np.array(val_array).reshape(schema_val.shape)
                d[k] = val
        return d
