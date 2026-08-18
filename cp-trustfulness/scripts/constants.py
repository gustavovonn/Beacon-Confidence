from dataclasses import dataclass
from enum import Enum
 
 
@dataclass(frozen=True)
class ExperimentDefaults:
    """Default parameter values for create_experiment_config."""
    start_time: int = 0
    time_limit: str = "35s"
    mode: str = "headless"
    fault_profile: str = "constant_offset"
    faulty_rate: float = 0.15
    seed: int = 0


class TimeOfDay(Enum):
    """Reference for time of day in seconds since midnight."""

    @staticmethod
    def hour_to_sec(hour: int) -> int:
        """Convert (24h-format) hour to seconds since midnight."""
        return hour * 3600

    MORNING   = hour_to_sec(7)
    RUSH_HOUR = hour_to_sec(7)
    AFTERNOON = hour_to_sec(12)
    EVENING   = hour_to_sec(17)
    NIGHT     = hour_to_sec(22)
