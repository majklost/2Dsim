from abc import ABC, abstractmethod

from ..messages.base_planner_messages import BasePlannerRequest, BasePlannerResponse
from ..simulators.base_simulator import Simulator, BaseSimulatorExport

class BasePlanner(ABC):
    def __init__(self, simulator: Simulator):
        self.simulator = simulator
    @abstractmethod
    def check_path(self, message: BasePlannerRequest) -> BasePlannerResponse:
        pass