# SSD Firmware Simulation and FTL Algorithm Implementation on STM32
**(基於 STM32 之 SSD 韌體模擬與 FTL 演算法實作)**

## 📝 專案簡介 (Project Overview)
本專案為一個基於 **C 語言** 與 **POSIX Threads (Pthreads)** 實作的雷達基頻訊號處理模擬系統。
針對 1T4R (單發四收) SIMO 架構，設計了一套 **Producer-Consumer Pipeline**。透過多執行緒技術，將高頻寬的ADC、低延遲的距離偵測 (Range DSP)，以及高運算量的角度估計 (DOA DSP) 進行解耦，完美模擬了真實雷達晶片中硬體加速器與 DSP Core 的協作模式，達成 Latency Hiding 與 Real-time Processing 的系統需求。

本專案在 **STM32F407 (Cortex-M4)** 平台上開發了一個輕量級的 SSD 韌體模擬器。核心目標是實作 NAND Flash 的底層管理邏輯，包含 **L2P table**、**Garbage Collection** 以及利用 **DMA 硬體加速** 來優化資料搬移效能。

## 🚀 核心技術與亮點 (Technical Highlights)
* **Page-level Mapping FTL:** 實作動態邏輯轉實體位址的 L2P Table。
* **Greedy Garbage Collection:** 採用 貪婪演算法 選取有效頁面最少的 Block 作為回收對象。
* **DMA 硬體搬移加速:** * 使用 DMA 處理資料搬移，大幅降低 CPU 負載。
* **Visual Debugging:** 
    * 🟢 綠燈：Host 端寫入中。

    * 🔴 紅燈：GC 搬移執行中。