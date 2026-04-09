# STM32 Multi-Channel SSD Firmware Simulator
**(基於 STM32 之 Multi-Channel SSD 韌體模擬)**

## 📝 專案簡介 (Project Overview)
本專案在 **STM32F407 (Cortex-M4)** 平台上實作一個的 SSD 韌體模擬器。核心目標是實作 NAND Flash 的底層管理邏輯，核心亮點在於將 **RTOS** 引入 FTL 管理邏輯，實現 **Host 讀取/寫入** 與 **Garbage Collection** 的並行處理，並透過 **Multi-channel Striping** 技術大幅提升 Throughput，除此之外，本專案實作了 **Hamming Code (ECC)** 錯誤偵測與自動修復機制，完整模擬 SSD 從 Host 端寫入、Flash 存放、到位元翻轉修復的數據全路徑。

## 🚀 核心技術與亮點 (Technical Highlights)
* **RTOS 多工任務調度 (Multi-tasking Architecture)**
    * **Foreground Host Task:** 處理前端 I/O 請求與 LBA 分流，確保高響應即時性。
    * **Background GC Task:** 獨立低優先級任務，自動檢查各通道剩下的有效 block 數量，在系統閒置時非同步執行回收，避免寫入阻塞。
* **多通道並行架構 (Multi-channel Striping)**
    * **Modular Striping:** 將 LBA 自動分佈至不同實體 Channel。
    * **獨立的 Free Block Pool 與管理邏輯:** 實現非同步並行處理。
* **FTL 與GC:**
    * **Page-level Mapping:** 透過動態 L2P Table 進行位址轉換。
    * **Greedy Garbage Collection:** 採用貪婪演算法選取有效頁面最少的 Block 作為回收對象。
    * **DMA 硬體搬移加速:** * 使用 DMA 處理資料搬移，大幅降低 CPU 負載。
* **全路徑數據保護 (End-to-End Data Protection)**
    * **Hamming Code (SEC) 整合:** 針對每個 Page 實作 Hamming(15, 11)，能在讀取時自動定位並修復 Single-bit Flip。
    * **Error Injection Simulator:** 內建隨機錯誤注入函式，模擬 NAND Flash 因電荷流失或干擾產生的物理錯誤，驗證韌體的自我修復能力。
    * **Read Reclaim 邏輯:** 在 Host Read 與 GC 搬移過程中強制執行 ECC 檢查，確保數據的正確性。
* **Visual Debugging:** 
    * 🟢 左燈 (Green)：Host 寫入中

    * 🟠 上燈 (Orange)：Host 讀取中

    * 🔴 右燈 (Red)：GC 觸發

    * 🔵 下燈 (Blue)：ECC 修復觸發
