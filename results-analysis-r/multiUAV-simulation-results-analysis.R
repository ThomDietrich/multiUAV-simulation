#!/usr/bin/Rscript
#
# CopterLogAnalysis.R - Statistical multicopter energy consumption
# analysis based on Ardupilot dataflash log files.
# Copyright (C) 2017  Thomas Dietrich
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# Clear workspace
remove(list = ls())
# Close all devices still open from previous executions
graphics.off()

#####################################################################
# Logfiles to analyze ###############################################

#####################################################################
# Settings ##########################################################

minSamplesForMean <- 30
transitionSampleSeconds <- 3

# Normalization
batteryVoltageCustomCopter   <- 3 * 3.7 # 11.1
batteryVoltageSoloCopter <- 4 * 3.7 # 14.8
referenceVoltage         <- 4 * 3.7

# http://www.stat.columbia.edu/~tzheng/files/Rcolor.pdf
#graphCol1 <- "steelblue"
#graphCol2 <- "goldenrod"
#graphCol2_dark <- "goldenrod4"
#graphCol3 <- "gray70"

#graphCol1 <- "#353594"
graphCol1 <- "#4682B4" #steelblue
graphCol2 <- "#B8850A" #mygold
graphCol2_dark <- "goldenrod4"
graphCol3 <- "#B3B3B3" #mygray
graphCol3_dark <- "#737373"
graphCol3_darkdark <- "#434343"
graphCol4 <- "#780116" #myred

#####################################################################
# Install packages, load libraries ##################################

library(data.table)
library(xtable)
list.of.packages <- c("Hmisc", "geosphere", "ggplot2", "cowplot", "tikzDevice", "TTR", "xts", "forecast", "stringr")
new.packages <- list.of.packages[!(list.of.packages %in% installed.packages()[,"Package"])]
if(length(new.packages)) install.packages(new.packages)
sapply(list.of.packages, require, character.only = TRUE)
lapply(list.of.packages, packageVersion)
remove(list = c("list.of.packages", "new.packages"))

# Increase max number of warnings
options(nwarnings=200) 

### Update packages (from time to time!)
#update.packages(checkBuilt=TRUE, ask=FALSE)

# plotlyUsername <- "user"
# plotlyApiKey <- "key"
#source("plotlyCredentials.R")

#Sys.setenv("plotly_username" = plotlyUsername)
#Sys.setenv("plotly_api_key" = plotlyApiKey)
#remove(plotlyUsername, plotlyApiKey)


theme_custom <- function () {
  theme_light() %+replace% 
    theme(
      panel.background  = element_blank(),
      axis.title = element_text(size = rel(0.8)),
      axis.ticks=element_blank(),
      legend.text = element_text(size = rel(0.8)),
      legend.title = element_text(size = rel(0.8)),
      panel.border=element_blank()
    )
}
theme_set(theme_custom())

options(tikzDefaultEngine = "xetex")
tikzLocation = "./tikz/"

#####################################################################
# Load functions files ##############################################

logdata_folder <- "../results/"

#overall OMNeT++ simulation time in seconds
simtimeSec <- 48 * 3600

df.all <- data.frame()

for (filename in list.files(logdata_folder,"*.sca")) {
  logdata.file <- data.frame()
  cat(filename)
  cat(": ")
  filename.replM <- NA
  filename.quant <- NA
  filename.numUAVs <- NA
  
  filename.basename <- str_replace(filename, "(.*?)-.*", "\\1")
  
  filename.replM   <- ifelse(grepl("replM=", filename), str_replace(filename, ".*-replM=(\\d).*", "\\1"), NA)
  filename.quant   <- ifelse(grepl("quant=", filename), str_replace(filename, ".*-quant=(\\d).*", "\\1"), NA)
  filename.numUAVs <- ifelse(grepl("numUAVs=", filename), str_replace(filename, ".*-numUAVs=(\\d).*", "\\1"), NA)
  
  filename.repeat <- str_replace(filename, ".*-#(\\d+).*", "\\1")
  
  cat("Load File ... ")
  lines <- readLines(paste(logdata_folder, filename, sep=""))
  #head(lines)
  lines.scalar <- grep("scalar OsgEarthNet.uav", lines, value=TRUE)
  #head(lines.scalar)
  
  cat("Parse Scalars ... ")
  for (line in lines.scalar) {
    uav.index <- str_replace(line, "scalar OsgEarthNet.uav\\[(\\d+)\\].*", "\\1")
    uav.metric <- str_replace(line, "scalar OsgEarthNet.uav\\[\\d+\\] (\\w*).*", "\\1")
    uav.metric.value <-str_replace(line, "scalar OsgEarthNet.uav\\[\\d+\\] \\w* (.*)", "\\1")
    #print(paste(uav.index, uav.metric, uav.metric.value, sep = " --- "))
    
    logdata.line <- data.frame(
      basename =         as.factor(filename.basename),
      replM =            as.integer(filename.replM),
      quant =            as.integer(filename.quant),
      numUAVs =          as.integer(filename.numUAVs),
      simRun =           as.integer(filename.repeat),
      uavIndex =         as.integer(uav.index),
      uavMetric =        as.factor(uav.metric),
      uavMetricValue =   as.numeric(uav.metric.value)
    )
    logdata.file <- rbind(logdata.file, logdata.line)
  }
  cat("\n")
  df.all <- rbind(df.all, logdata.file)
}

head(df.all)

sim_runs <- unique(df.all$simRun)
replMs <- unique(df.all$replM)
metrics <- levels(factor(df.all$uavMetric))
uavs_count <- max(df.all$uavIndex)



cat("Remove UAVs with 'fail' metric ... ")

uavs_all <- subset(df.all, uavMetric == 'utilizationFail')
uavs_all_failed <- subset(df.all, uavMetric == 'utilizationFail' & uavMetricValue != 0)

if (nrow(uavs_all_failed) > 0) {
  warning(paste(nrow(uavs_all_failed), "of", nrow(uavs_all), "UAVs over all simulation runs ended in 'fail' state", sep=" "))
}

for(i in 1:nrow(uavs_all_failed)) {
  row <- uavs_all_failed[i,]
  #print(row)
  df.all <- df.all[!(df.all$replM==row$replM & df.all$simRun==row$simRun & df.all$uavIndex==row$uavIndex),]
}



cat("Proofchecking Simtime ... ")
tolerance = 0.1
for (replMethod in replMs) {
  for (sim_run in sim_runs) {
    for (index in 0:uavs_count) {
      df_subset <- subset(df.all, replM == replMethod & simRun == sim_run & uavIndex == index
                          & uavMetric %in% c('utilizationSecMaintenance', 'utilizationSecMission', 'utilizationSecCharge', 'utilizationSecIdle')
      )
      if (nrow(df_subset) > 0) {
        #print(sum(df_subset$uavMetricValue))
        if (abs(simtimeSec - sum(df_subset$uavMetricValue)) > tolerance) warning("Simtime missmatch!")
      }
    }
  }
}



cat("Combining SimRuns ... ")
df.red.simrun <- data.frame()

for (replMethod in replMs) {
  for (index in 0:uavs_count) {
    for (metric in metrics) {
      df_subset <- subset(df.all, replM == replMethod & uavIndex == index & uavMetric == metric)
      metric_value.mean <- sum(df_subset$uavMetricValue)
      metric_value.stddev <- sqrt(var(df_subset$uavMetricValue))
      #print(paste(replMethod, uav, metric, metric_value.mean, metric_value.stddev, sep = " -- "))
      df <- data.frame(
        basename =    levels(df_subset$basename),
        replM =        as.integer(replMethod),
        uavIndex =    as.integer(index),
        uavMetric =   as.factor(metric),
        valueMean =   as.numeric(metric_value.mean),
        valueStddev = as.numeric(metric_value.stddev)
      )
      df.red.simrun <- rbind(df.red.simrun, df)
    }    
  }
}
#remove(replMethod, uav, metric, df_subset, metric_value.mean, metric_value.stddev, df)



cat("Summarizing Metrics per UAV... ")
df.red <- data.frame()

for (replMethod in replMs) {
  for (index in 0:uavs_count) {
    df_subset <- subset(df.red.simrun, replM == replMethod & uavIndex == index)
    df <- data.frame(
      basename =             levels(df_subset$basename),
      replM =                 as.integer(replMethod),
      uavIndex =             as.integer(index),
      #
      secMission =                  as.numeric(subset(df_subset, uavMetric == 'utilizationSecMission')$valueMean),
      secMaintenance =              as.numeric(subset(df_subset, uavMetric == 'utilizationSecMaintenance')$valueMean),
      secCharge =                   as.numeric(subset(df_subset, uavMetric == 'utilizationSecCharge')$valueMean),
      secIdle =                     as.numeric(subset(df_subset, uavMetric == 'utilizationSecIdle')$valueMean),
      #
      energyMission =               as.numeric(subset(df_subset, uavMetric == 'utilizationEnergyMission')$valueMean),
      energyMaintenance =           as.numeric(subset(df_subset, uavMetric == 'utilizationEnergyMaintenance')$valueMean),
      energyCharge =                as.numeric(subset(df_subset, uavMetric == 'utilizationEnergyCharge')$valueMean),
      #
      energyOverdrawMission =       as.numeric(subset(df_subset, uavMetric == 'utilizationEnergyOverdrawMission')$valueMean),
      energyOverdrawMaintenance =   as.numeric(subset(df_subset, uavMetric == 'utilizationEnergyOverdrawMaintenance')$valueMean),
      #
      countMissions =               as.numeric(subset(df_subset, uavMetric == 'utilizationCountMissions')$valueMean),
      countManeuversMission =       as.numeric(subset(df_subset, uavMetric == 'utilizationCountManeuversMission')$valueMean),
      countManeuversMaintenance =   as.numeric(subset(df_subset, uavMetric == 'utilizationCountManeuversMaintenance')$valueMean),
      countChargeState =            as.numeric(subset(df_subset, uavMetric == 'utilizationCountChargeState')$valueMean),
      countOverdrawnAfterMission =  as.numeric(subset(df_subset, uavMetric == 'utilizationCountOverdrawnAfterMission')$valueMean),
      countIdleState =              as.numeric(subset(df_subset, uavMetric == 'utilizationCountIdleState')$valueMean)
    )
    df.red <- rbind(df.red, df)
  }    
}



cat("Draw ... ")

ggplot(df.red) +
  geom_bin2d(aes(x = 100 / (secMission + secMaintenance) * secMission, y = 100 / (secMission + secMaintenance) * secMaintenance)) +
  labs(x="Time in Mission", y="Time in Maintenance")

ggplot(df.red) +
  geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) +
  labs(x="Energy in Mission", y="Energy in Maintenance")


ggplot(subset(df.red, replM==0)) + geom_bin2d(aes(x = energyMission, y = energyMaintenance)) + ylim(50000, 200000) + xlim(50000, 250000)
ggplot(subset(df.red, replM==1)) + geom_bin2d(aes(x = energyMission, y = energyMaintenance)) + ylim(50000, 200000) + xlim(50000, 250000)
ggplot(subset(df.red, replM==2)) + geom_bin2d(aes(x = energyMission, y = energyMaintenance)) + ylim(50000, 200000) + xlim(50000, 250000)

ggplot(subset(df.red, replM==0)) + geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) + ylim(25, 80) + xlim(25, 70)
ggplot(subset(df.red, replM==1)) + geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) + ylim(25, 80) + xlim(25, 70)
ggplot(subset(df.red, replM==2)) + geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) + ylim(25, 80) + xlim(25, 70)
