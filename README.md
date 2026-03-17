# STM32 Multi-Channel SSD Firmware Simulator
**(基於 RTOS 多工架構與 DMA 加速之 Flash Memory 管理系統)**

## 📝 專案簡介 (Project Overview)
本專案在 **STM32F407 (Cortex-M4)** 平台上實作一個的 SSD 韌體模擬器。核心目標是實作 NAND Flash 的底層管理邏輯，核心亮點在於將 **RTOS** 引入 FTL 管理邏輯，實現 **Host 寫入** 與 **Garbage Collection** 的並行處理，並透過 **Multi-channel Striping** 技術大幅提升 Throughput。

## 🚀 核心技術與亮點 (Technical Highlights)
**1. RTOS 多工任務調度 (Multi-tasking Architecture)**
    * **Foreground Host Task:** 處理前端 I/O 請求與 LBA 分流，確保高響應即時性。
    * **Background GC Task:** 獨立低優先級任務，自動檢查各通道剩下的有效 block 數量，在系統閒置時非同步執行回收，避免寫入阻塞。
**2. 多通道並行架構 (Multi-channel Striping)**
    * **Modular Striping:** 將 LBA 自動分佈至不同實體 Channel。
    * **獨立的 Free Block Pool 與管理邏輯:** 實現非同步並行處理。
**3. FTL 與GC:**
    * **Page-level Mapping:** 透過動態 L2P Table 進行位址轉換。
    * **Greedy Garbage Collection:** 採用 貪婪演算法 選取有效頁面最少的 Block 作為回收對象。
    * **DMA 硬體搬移加速:** * 使用 DMA 處理資料搬移，大幅降低 CPU 負載。
**4. Visual Debugging:** 
    * 🟢 左燈 (Green)：Channel 0 - Host 寫入中

    * 🟠 上燈 (Orange)：Channel 1 - Host 寫入中

    * 🔴 右燈 (Red)：Channel 0 - 後台 GC 搬移中

    * 🔵 下燈 (Blue)：Channel 1 - 後台 GC 搬移中
