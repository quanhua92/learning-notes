# Generative Adversarial Network

- Generate realistic-looking images from scratch

## Randomness

Neural network will always give the same output if we feed them the same input.

Instead of adding the randomness into the network, we will sample our input z from a random distribution `pz(z)`.

## Type of networks

Discriminator vs Generator

## Loss function

- For x from real work: loss function = `-log(D(x))` . --> -log(1) = 0 -> D(x) = 1 -> no loss
- For x from generator: loss function = `-log(1 - D(G(z))` -> -log(1 - 0) = 0 -> D(x) = 0 -> no loss