#! /usr/bin/python

import os
import Queue
import threading
import shutil
import subprocess
import sys


class Worker(threading.Thread):
    def __init__(self, queue, success, failure):
        threading.Thread.__init__(self)
        self.__queue = queue
        self.__success = success
        self.__failure = failure

    def run(self):
        while True:
            item = self.__queue.get()
            if item == 'end':
                self.__queue.task_done()
                return

            # Run the test
            fout = open('results/' + item.replace('/', '_'), 'w')
            ret = subprocess.call(['/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout', '600',
                                   '/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4',
                                   '-distro',
                                   '-L', '/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target',
                                   '-x', os.getcwd(), '-cwd', os.getcwd(),
                                   'obj/test/debug/cctest',
                                   item], stdout=fout, stderr=subprocess.STDOUT)

            fout.close()
            message = '=== ' + item + ' ==='
            # Remove the file if the result is 0 (success)
            if ret == 0:
                os.remove('results/' + item.replace('/', '_'))
                self.__success.append(item)
            else:
                self.__failure.append(item)
                message = message + '\n...FAILED'

            print(message)
            self.__queue.task_done()


def main():
    success = []
    failure = []
    queue = Queue.Queue()
    workers = 4
    result_dir = 'results'

    if len(sys.argv) == 3:
        workers = int(sys.argv[1])
        result_dir = sys.argv[2]

    shutil.rmtree(result_dir)
    os.mkdir(result_dir)

    # launch the workers
    for i in range(workers):
         w = Worker(queue, success, failure)
         w.start()

    # List the test and add them to the list of task
    list_tests = subprocess.Popen(['/sw/st/gnu_compil/gnu/linux-rh-ws-4/bin/timeout', '600',
                                   '/home/compwork/guillon/qemu-stm/build-x86_64/devimage/bin/qemu-sh4',
                                   '-distro',
                                   '-L', '/home/compwork/projects/stlinux/opt/STM/STLinux-2.3/devkit/sh4/target',
                                   '-x', os.getcwd(), '-cwd', os.getcwd(),
                                   'obj/test/debug/cctest',
                                   '--list'],
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT)

    for line in list_tests.communicate()[0].split('\n'):
        if line.find('test-') == 0:
            queue.put(line[:line.find('<')])

    # kill the workers and wait for them
    for i in range(workers):
        queue.put('end')
    queue.join()

    print('=========================')
    print('Failed test: ' + str(len(failure)) + '/' + str(len(failure) + len(success)))
    print('=========================')

    failure.sort()
    if len(failure) > 0:
        print('')
        print('failures:')
        for fail in failure:
            print(' * ' + fail)

# Main entry point
if __name__ == '__main__':
    main()


