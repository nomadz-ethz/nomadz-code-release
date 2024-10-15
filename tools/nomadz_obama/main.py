#!/usr/bin/env python3
import threading
import time
from typing import Callable, Mapping

from backend import Backend, Robot, RobotStatus
from drag_and_drop import Card, DragHandler, EmptyCard, Item, Row
from nicegui import Client, ui
from returns.result import Failure, Result, Success


class App:
    def __init__(self):
        self.backend = Backend()
        self.draghandler = DragHandler()
        self.robot_cards_selected = []
        self.init_robot_entities()

    def init_robot_entities(self):
        self.robot_entities = {
            Robot("Dain", "10.0.33.182", 82): RobotStatus(),
            Robot("Thror", "10.0.33.183", 83): RobotStatus(),
            Robot("Dis", "10.0.33.184", 84): RobotStatus(),
            Robot("Borin", "10.0.33.185", 85): RobotStatus(),
            Robot("Farin", "10.0.33.186", 86): RobotStatus(),
            Robot("Pablo", "10.0.33.187", 87): RobotStatus(),
            Robot("Koshka", "10.0.33.188", 88): RobotStatus(),
            Robot("Lyra", "10.0.33.189", 89): RobotStatus(),
            Robot("Diablo", "10.0.33.190", 90): RobotStatus(),
            Robot("Nyx", "10.0.33.191", 91): RobotStatus(),
        }

    def get_robot_cards_selected(self, robot_cards: list[Card]) -> list[Card]:
        """returns a list of cards which are selected using the checkbox

        @param robot_cards: all robot cards which are initialized in this gui
        @type robot_cards: list[Card]
        @return: list of the selected cards
        @rtype: list[Card]
        """
        robot_cards_selected = [card for card in robot_cards if card.item.selected]
        return robot_cards_selected

    def periodic_check(self, robot_cards: list[Card]) -> None:
        """a function which will perform defined checks on the robots in a periodic manner
        @param robot_cards: a list of robot cards
        @type robot_cards: list[Card]
        @return: None
        @rtype: None
        """

        def check():
            while True:
                self.apply_backend(robot_cards, self.backend.get_connection)
                self.apply_backend(robot_cards, self.backend.get_battery_status)
                time.sleep(5)

        threading.Thread(target=check, daemon=True).start()

    def update_robot_entities(self, robot_cards: list[Card], field: str) -> None:
        """function to update the field saved in the backend data of the robots (robot_entities)
        @param robot_cards: a list of robot cards
        @type robot_cards: list[Card]
        @param field: the field to which the robots are deployed
        @type field: str
        @return: None
        @rtype: None
        """
        self.robot_entities_selected = {
            robot: status
            for robot, status in self.robot_entities.items()
            if any(robot.name == card.item.title for card in robot_cards)
        }
        for robot in self.robot_entities_selected:
            self.robot_entities_selected[robot].field = field.value
            print(self.robot_entities_selected[robot].field)
        ui.notify(
            f"updated field of {str(len(self.robot_entities_selected))} robots: {[robot.name for robot in self.robot_entities_selected]} to {field.value}"
        )

    def update_robot_role(self, robot_cards: list[Card]) -> None:
        """
        function to update the role of the robots saved in the backend data of the robots (robot_entities)
        @param robot_cards: a list of robot cards
        @type robot_cards: list[Card]
        @return: None
        @rtype: None
        """
        for robot in self.robot_entities:
            for card in robot_cards:
                if card.item.title != robot.name:
                    continue
                self.robot_entities[robot].role = card.item.role
        self.apply_backend(robot_cards, self.backend.set_role)

    def update_player_id(self, robot_cards: list[Card]) -> None:
        """
        function to update the player_id of the robots saved in the backend data of the robots (robot_entities)
        @param robot_cards: a list of robot cards
        @type robot_cards: list[Card]
        @return: None
        @rtype: None
        """

        for robot in self.robot_entities:
            for card in robot_cards:
                if card.item.title != robot.name:
                    continue
                self.robot_entities[robot].player_id = card.item.number
        self.apply_backend(robot_cards, self.backend.set_player_id)

    def update_robot_cards(
        self, robots_entities: Mapping[Robot, RobotStatus], robot_cards: list[Card]
    ) -> None:
        """
        function to update the robot cards with the data from the periodic checks
        @param robots_entities: a dictionary of robot entities
        @type robots_entities: Mapping[Robot, RobotStatus]
        @param robot_cards: a list of robot cards
        @type robot_cards: list[Card]
        @return: None
        @rtype: None
        """
        for robot in robots_entities:
            for card in robot_cards:
                if card.item.title != robot.name:
                    continue
                card.item.battery_status = robots_entities[robot].battery_status
                card.item.connection_lan = robots_entities[robot].connection_lan
                card.item.connection_wlan = robots_entities[robot].connection_wlan
                card.update_card()

    def apply_backend(self, robot_cards: list[Card], func: Callable) -> None:
        """
        function to apply the backend functions on the selected robots
        @param robot_cards: a list of robot cards
        @type robot_cards: list[Card]
        @param func: a function to be applied on the selected robots
        @type func: Callable
        @return: None
        @rtype: None
        """
        self.robot_entities_selected = {
            robot: status
            for robot, status in self.robot_entities.items()
            if any(robot.name == card.item.title for card in robot_cards)
        }
        result: Result = func(self.robot_entities_selected)

        match result:
            case Success(None):
                # TODO notify?
                ui.notify("Success")
                pass
            case Success(self.robot_entities_selected):
                self.update_robot_cards(self.robot_entities_selected, robot_cards)
                pass
            case Failure(blame):
                ui.notify(f"Failure {blame}")
                # TODO notify in UI Failure? or display in a separate container in the UI?
                pass
            case Failure(None):
                ui.notify("Failure")
                # TODO notify in UI Failure? or display in a separate container in the UI?
                pass

    def setup(self):
        Client.auto_index_client
        with ui.row().classes("items-stretch w-full no-wrap"):
            with ui.card().classes("w-4/5 p-4 rounded shadow-2"):
                with Row("Field").classes("place-content-center"):
                    # row of the deploying robots
                    ui.separator()
                    EmptyCard(self.draghandler)
                    EmptyCard(self.draghandler)
                    EmptyCard(self.draghandler)
                    EmptyCard(self.draghandler)
                    EmptyCard(self.draghandler)
                    EmptyCard(self.draghandler)
                    EmptyCard(self.draghandler)
                with Row("Robots").classes("place-content-center"):
                    # row of the robots available
                    ui.separator()
                    robot_cards = [
                        Card(Item("Thror", 1), self.draghandler),
                        Card(Item("Nyx", 2), self.draghandler),
                        Card(Item("Pablo", 3), self.draghandler),
                        Card(Item("Koshka", 4), self.draghandler),
                        Card(Item("Diablo", 5), self.draghandler),
                        Card(Item("Borin", 6), self.draghandler),
                        Card(Item("Farin", 7), self.draghandler),
                        Card(Item("Dis", 8), self.draghandler),
                        Card(Item("Lyra", 9), self.draghandler),
                        Card(Item("Dain", 10), self.draghandler),
                    ]
            with ui.card().classes(
                "w-1/5 p-4 rounded text-center items-center items-stretch"
            ):
                ui.label("Setup")
                ui.select(
                    ["NomadZ", "SPL_A", "SPL_B", "SPL_C", "SPL_D", "SPL_E"],
                    label="Field",
                    value="NomadZ",
                    on_change=lambda value: self.update_robot_entities(
                        self.get_robot_cards_selected(robot_cards), value
                    ),
                )
                ui.button(
                    "Deploy",
                    on_click=lambda: self.apply_backend(
                        self.get_robot_cards_selected(robot_cards),
                        self.backend.deploy,
                    ),
                )
                ui.button(
                    "load wifi config",
                    on_click=lambda: self.apply_backend(
                        self.get_robot_cards_selected(robot_cards),
                        self.backend.load_wifi_config,
                    ),
                )
                ui.button(
                    "load new roles",
                    on_click=lambda: self.update_robot_role(
                        self.get_robot_cards_selected(robot_cards)
                    ),
                )
                ui.button(
                    "update player id",
                    on_click=lambda: self.update_player_id(
                        self.get_robot_cards_selected(robot_cards)
                    ),
                )
                ui.separator()

                ui.button(
                    "Restart nomadz",
                    on_click=lambda: self.apply_backend(
                        self.get_robot_cards_selected(robot_cards),
                        self.backend.restart,
                    ),
                )
                ui.button(
                    "Reboot",
                    on_click=lambda: self.apply_backend(
                        self.get_robot_cards_selected(robot_cards),
                        self.backend.reboot,
                    ),
                )
                ui.button(
                    "Shutdown",
                    on_click=lambda: self.apply_backend(
                        self.get_robot_cards_selected(robot_cards),
                        self.backend.shutdown,
                    ),
                )
                ui.button(
                    "Download logs",
                    on_click=lambda: self.apply_backend(
                        self.get_robot_cards_selected(robot_cards),
                        self.backend.download_logs,
                    ),
                )
        self.periodic_check(robot_cards)


app = App()

app.setup()
ui.run(favicon="🤖", title="Obama", reload=False)
