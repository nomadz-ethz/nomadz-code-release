from __future__ import annotations

from dataclasses import dataclass
from functools import partial
from typing import Callable, Optional

from nicegui import ui


@dataclass
class Item:
    title: str
    number: int
    role: str = "Player"
    battery_status: float = 0.0
    connection_lan: bool = False
    connection_wlan: bool = False
    selected: bool = False


class DragHandler:
    dragged: Optional[Card] = None


class Row(ui.row):
    def __init__(
        self, name: str, on_drop: Optional[Callable[[Item, str], None]] = None
    ) -> None:
        super().__init__()
        with self.classes("bg-blue-grey-2 w-full p-4 rounded shadow-2"):
            ui.label(name).classes("text-bold ml-1")
        self.name = name
        self.on_drop = on_drop


class EmptyCard(ui.card):
    def __init__(self, draghandler: DragHandler) -> None:
        """initialize a base card with some default properties and event handlers"""
        super().__init__()
        self.classes("w-1/5 h-44 bg-grey-2")
        self.on("dragover.prevent", self.highlight)
        self.on("dragleave", self.unhighlight)
        self.on("drop", self.move_card)
        self.draghandler = draghandler

    def highlight(self) -> None:
        """highlight the card when hovering over it with a draggable card"""
        self.classes(remove="bg-grey-2", add="bg-grey-3")

    def unhighlight(self) -> None:
        """reverse of highlight"""
        self.classes(remove="bg-grey-3", add="bg-grey-2")

    def move_card(self):
        """move the card to the new location"""
        idx = self.parent_slot.children.index(self)
        old_dragged_parent_slot_parent = self.draghandler.dragged.parent_slot.parent
        old_idx = self.draghandler.dragged.parent_slot.children.index(
            self.draghandler.dragged
        )
        self.draghandler.dragged.move(self.parent_slot.parent, idx)
        self.move(old_dragged_parent_slot_parent, old_idx)
        self.handle_drop(self.parent_slot.parent)
        self.handle_drop(self.draghandler.dragged.parent_slot.parent)
        self.unhighlight()

    @staticmethod
    def handle_drop(row: Row):
        """event handler that updates the card numbers when a card is dropped
        @param row: the row where the card was dropped and therefore the robot list should be extended
        """
        robot_list = []
        robot_idx = 1
        for elem in row.default_slot.children:
            if isinstance(elem, EmptyCard):
                robot_list.append((robot_idx, elem))
                robot_idx += 1

        for idx, elem in robot_list:
            if hasattr(elem, "update_card"):
                elem.item.number = idx

                elem.update_card()


class Card(EmptyCard):
    def __init__(self, item: Item, draghandler: DragHandler) -> None:
        """adds additional properties to the base card to add information about robots"""
        super().__init__(draghandler)
        self.item = item
        self.update_card()
        self.on("dragstart", partial(self.handle_dragstart, draghandler))

    def update_card(self) -> None:
        """update the card with the new information"""
        self.clear()
        item = self.item
        with self.props("draggable").classes("w-1/5 h-44 cursor-pointer bg-grey-1"):
            with ui.row().classes("w-full"):
                ui.label(item.title).classes("text-bold text-base mr-2")
                ui.label("|").classes("text-bold text-base")
                ui.label(item.number).classes("text-bold text-base ml-2")
                ui.space()
                ui.icon(
                    "r_signal_cellular_alt",
                    color="#00FF00" if item.connection_wlan else "#FF0000",
                ).classes("text-base text-xl")
                ui.icon(
                    "r_settings_ethernet",
                    color="#00FF00" if item.connection_lan else "#FF0000",
                ).classes("text-base text-xl")
            ui.linear_progress(value=self.item.battery_status, color="#00FF00").props(
                "instant-feedback rounded stripe"
            )
            ui.html("<style>.q-linear-progress{border-radius: 5px !important;}</style>")
            with ui.row().classes("w-full"):
                ui.select(
                    ["Player", "Defender", "Mr. Li s RL policy ", "Goalkeeper"],
                    label="Role",
                    value=self.item.role,
                    on_change=self.update_role,
                )
                ui.space()
                ui.checkbox(
                    on_change=self.update_checkbox,
                    value=self.item.selected,
                ).tooltip("Select robot for group actions")

    def update_role(self, update_event: str) -> None:
        """update the role of the robot"""
        self.item.role = update_event.value

    def handle_dragstart(self, draghandler: DragHandler) -> None:
        draghandler.dragged = self

    def update_checkbox(self, update_event: bool):
        self.item.selected = update_event.value
