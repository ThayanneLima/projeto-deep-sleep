import pandas as pd

arquivo = "dados deep sleep.csv"

# 1) tentar ler como UTF-16 (muito comum quando foi salvo pelo Excel)
try:
    df = pd.read_csv(
        arquivo,
        sep=r"\s+",
        encoding="utf-16",
        skiprows=1,
        names=["date","time","Battery_current_ma","Battery_voltage_v","Energy_wh","OperationMode"],
        engine="python",
        on_bad_lines="skip"
    )
except UnicodeError:
    # 2) se falhar, tenta latin1
    df = pd.read_csv(
        arquivo,
        sep=r"\s+",
        encoding="latin1",
        skiprows=1,
        names=["date","time","Battery_current_ma","Battery_voltage_v","Energy_wh","OperationMode"],
        engine="python",
        on_bad_lines="skip"
    )

# transformar colunas em número (se vier texto, vira NaN e não quebra)
for c in ["Battery_current_ma", "Battery_voltage_v", "Energy_wh", "OperationMode"]:
    df[c] = pd.to_numeric(df[c], errors="coerce")

# remover linhas sem dados válidos
df = df.dropna(subset=["Battery_current_ma", "Battery_voltage_v", "Energy_wh"])

print("Média corrente (mA):", df["Battery_current_ma"].mean())
print("Média tensão (V):", df["Battery_voltage_v"].mean())
print("Média energia (Wh):", df["Energy_wh"].mean())