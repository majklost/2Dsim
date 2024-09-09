from abc import ABC, abstractmethod

from ..messages.base_planner_messages import BasePlannerRequest, BasePlannerResponse

class BasePlanner(ABC):
    @abstractmethod
    def check_path(self, message: BasePlannerRequest) -> BasePlannerResponse:
        pass