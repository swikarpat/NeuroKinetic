package com.neurokinetic.gateway.controller;

import jakarta.validation.Valid;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import reactor.core.publisher.Mono;

import java.time.Instant;
import java.util.List;
import java.util.UUID;

@RestController
@RequestMapping("/api/v1/gateway")
public class WorkOrderController {

    public record WorkOrderRequest(
        @NotBlank String orderId,
        @NotBlank String directive,
        @NotNull List<Double> targetJoints
    ) {}

    public record WorkOrderResponse(
        String dispatchId,
        String status,
        String assignedAsset,
        long timestamp
    ) {}

    @PostMapping("/work-orders")
    public Mono<ResponseEntity<WorkOrderResponse>> ingestWorkOrder(@Valid @RequestBody WorkOrderRequest request) {
        String dispatchId = UUID.randomUUID().toString();
        System.out.println("[Java Netty Gateway] Ingested Enterprise Work Order: " + request.orderId() + " | Directive: " + request.directive());
        
        WorkOrderResponse response = new WorkOrderResponse(
            dispatchId,
            "DISPATCHED_TO_EDGE_MESH",
            "ROBOT-ALPHA",
            Instant.now().toEpochMilli()
        );

        return Mono.just(ResponseEntity.accepted().body(response));
    }

    @GetMapping("/health")
    public Mono<ResponseEntity<String>> healthCheck() {
        return Mono.just(ResponseEntity.ok("JAVA 21 NETTY GATEWAY ACTIVE : PROTOCOL OT/IT BRIDGE OK"));
    }
}
