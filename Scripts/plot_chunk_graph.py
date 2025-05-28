#!/usr/bin/env python3

import pandas
import matplotlib.pyplot as plt

df = pandas.read_csv('test_results.csv')

chunk_df = df[df["chunk_size"] > 0]
normal_df = df[df["chunk_size"] == 0]

plt.figure(figsize=(12, 6))
bar_width = 0.35
index = range(len(chunk_df))

plt.bar(
    index,
    chunk_df["total_time"],
    bar_width,
    label="normal",
    color="blue"
)

plt.bar(
    [i + bar_width for i in index],
    normal_df["total_time"],
    bar_width,
    label="chunk",
    color="red"
)

plt.xlabel("Image")
plt.ylabel("Total Time (s)")
plt.title("Total Time for Chunk vs Normal Transfer")

plt.xticks(
    [i + bar_width / 2 for i in index],
    chunk_df["image_path"],
    rotation=45,
    ha="right"
)

plt.legend()
plt.tight_layout()
plt.savefig("chunk_vs_normal.png", dpi=300)
plt.show()
