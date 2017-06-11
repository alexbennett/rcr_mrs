#!/usr/bin/env python

from mrs import Logger
import time

logger = Logger('output')

logger.log_data('Data test')
logger.log_info('Info test')
logger.log_warning('Warning test')
logger.log_error('Error test')

logger.close()
