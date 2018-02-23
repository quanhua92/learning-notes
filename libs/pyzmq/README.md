1. Install pyzmq

   ```http://zeromq.org/bindings:python```

   ```bash
   pip install pyzmq
   ```

2. Code and examples: [https://learning-0mq-with-pyzmq.readthedocs.io/en/latest/pyzmq/pyzmq.html](https://learning-0mq-with-pyzmq.readthedocs.io/en/latest/pyzmq/pyzmq.html)

3. Example 01: [Push / Pull pattern](https://learning-0mq-with-pyzmq.readthedocs.io/en/latest/pyzmq/patterns/pushpull.html)

- Producer: for example leap motion process

```python
# producer.py
import time
import zmq

def producer():
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUSH)
    zmq_socket.bind("tcp://127.0.0.1:5557")
    # Start your result manager and workers before you start your producers
    for num in range(20000):
        # eg. Read leap motion data and create a json `work_message`
        work_message = { 'num' : num }
        zmq_socket.send_json(work_message)

producer()
```

- Consumer: for example simulation process

```python
# consumer.py
import time
import zmq
import random

def consumer():
    consumer_id = random.randrange(1,10005)
    print("I am consumer # %s" % (consumer_id))
    context = zmq.Context()
    # recieve work
    consumer_receiver = context.socket(zmq.PULL)
    consumer_receiver.connect("tcp://127.0.0.1:5557")
    
    while True:
        work = consumer_receiver.recv_json()
        data = work['num']
        print(data)
        # Animate the hand in simulation
        
consumer()
```

4. Open 2 terminal. Run consumer.py first then run producer.py

```bash
python consumer.py
```

```bash
python producer.py
```



