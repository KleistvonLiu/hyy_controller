import numpy as np
import matplotlib.pyplot as plt

def load_txt_csv_like(path: str) -> np.ndarray:
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            toks = [t.strip() for t in line.split(",") if t.strip() != ""]
            if toks:
                rows.append([float(t) for t in toks])
    arr = np.array(rows, dtype=np.float64)
    if arr.shape[1] < 14:
        raise ValueError(f"Need at least 14 columns, got {arr.shape[1]}")
    return arr[:, :14]  # 只取前14列：0..13

def plot_7_pairs_one_figure(txt_path: str):
    data = load_txt_csv_like(txt_path)

    # 创建一个大图，包含 7 行 1 列子图
    fig, axes = plt.subplots(7, 1, figsize=(28, 10), sharex=True)
    x = np.arange(data.shape[0])

    for i in range(7):
        ax = axes[i]
        ax.plot(x, data[:, i],     label=f"col[{i}]")
        ax.plot(x, data[:, i + 7], label=f"col[{i+7}]")
        ax.set_ylabel("value")
        ax.set_title(f"Column {i} vs Column {i+7}")
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("row index")
    fig.tight_layout()
    plt.show()

def main():
    # txt_path = "/home/kleist/Documents/Code/temp/servotest.txt"
    txt_path = "/home/kleist/Documents/Code/temp/jeserver.txt"
    plot_7_pairs_one_figure(txt_path)

if __name__ == "__main__":
    main()
