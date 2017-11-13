# Intelligence Module
The Intelligence module covers any kind of machine learning, deep learning, neural networking or AI, in which research and software have made dramatic leaps forward in recent times.  
- [**Tensorflow**](/modules/intelligence#tensorflow): A hugely important libary from Google for machine intelligence.

### Tensorflow
Tensorflow is installed by default as a python library, simply `import tensorflow` in python to start using it.  There is lots of great developer information on the [Tensorflow website](https://www.tensorflow.org/get_started/) to get you started.

Tensorflow can take a *very* long time to compile on source-required platforms (ARM).  Compiling tensorflow on a Raspberry Pi takes the best part of a day, so bootstrapping Maverick can appear to hang at this stage.  Since Maverick 1.1.4 experimental binary ARM packages are used which considerably speeds up the build process. To turn off tensorflow compile, add a localconf parameter:  
`"maverick_intelligence::tensorflow": false`  
