import logging


class NavigationMaxTimeError(Exception):
    time: int = 100

    def __init__(self, logger: logging.Logger, message: str):
        logger.error(message)
