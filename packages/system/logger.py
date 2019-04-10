import logging


def logger(name, level="info", format="%(asctime)-15s  %(levelname)-6s  %(message)s"):
  logger = logging.getLogger(name)
  logger.setLevel(getattr(logging, level.upper()))

  handler = logging.StreamHandler()
  handler.setLevel(getattr(logging, level.upper()))

  formatter = logging.Formatter(format)
  handler.setFormatter(formatter)

  logger.addHandler(handler)

  return logger
